// Unit tests for EpisodeSystemHardware with the MOCK backend (Synthetic / Mock Data).
// The component is driven directly (init -> export -> configure -> activate ->
// read/write), without a controller_manager.

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "episode_system_hardware.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using episode_ros2_control::EpisodeSystemHardware;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

std::string make_urdf(int n_joints, const std::string & signs = "1,1,1,1,1,1")
{
  std::string urdf = R"(<?xml version="1.0"?><robot name="t"><link name="base"/>)";
  std::string parent = "base";
  for (int i = 1; i <= n_joints; ++i) {
    std::string l = "l" + std::to_string(i);
    urdf += "<link name=\"" + l + "\"/><joint name=\"joint" + std::to_string(i) +
      "\" type=\"revolute\"><parent link=\"" + parent + "\"/><child link=\"" + l +
      "\"/><axis xyz=\"0 0 1\"/><limit lower=\"-6\" upper=\"6\" effort=\"1\" velocity=\"1\"/></joint>";
    parent = l;
  }
  urdf += R"(<ros2_control name="Episode1System" type="system"><hardware>
    <plugin>episode_ros2_control/EpisodeSystemHardware</plugin>
    <param name="backend">mock</param>
    <param name="mapping_signs">)" + signs + R"(</param>
    <param name="mapping_offsets_deg">180,90,83,30,110,30</param>
    <param name="joint_lower_deg">0,0,0,0,0,0</param>
    <param name="joint_upper_deg">340,180,163,335,220,335</param>
    <param name="speed_ratio">1.0</param>
    <param name="min_command_interval_s">0.0</param>
    </hardware>)";
  for (int i = 1; i <= n_joints; ++i) {
    urdf += "<joint name=\"joint" + std::to_string(i) +
      "\"><command_interface name=\"position\"/><state_interface name=\"position\"/>"
      "<state_interface name=\"velocity\"/></joint>";
  }
  urdf += "</ros2_control></robot>";
  return urdf;
}

class EpisodeHwTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  CallbackReturn init_with(const std::string & urdf)
  {
    auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = infos.at(0);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params.logger = rclcpp::get_logger("test_episode_hw");
    return hw_.init(params);
  }

  void export_all()
  {
    for (auto & s : hw_.on_export_state_interfaces()) {states_[s->get_name()] = s;}
    for (auto & c : hw_.on_export_command_interfaces()) {commands_[c->get_name()] = c;}
  }

  double state(const std::string & n) {return states_.at(n)->get_optional<double>().value();}
  void command(const std::string & n, double v) {ASSERT_TRUE(commands_.at(n)->set_value(v));}

  return_type cycle()
  {
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
    auto r = hw_.read(rclcpp::Time(0, 0, RCL_STEADY_TIME), dt);
    if (r != return_type::OK) {return r;}
    return hw_.write(rclcpp::Time(0, 0, RCL_STEADY_TIME), dt);
  }

  EpisodeSystemHardware hw_;
  std::map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
  rclcpp_lifecycle::State st_;
};

TEST_F(EpisodeHwTest, rejects_wrong_joint_count)
{
  EXPECT_EQ(init_with(make_urdf(5)), CallbackReturn::ERROR);
}

TEST_F(EpisodeHwTest, rejects_bad_sign)
{
  EXPECT_EQ(init_with(make_urdf(6, "1,1,1,1,1,2")), CallbackReturn::ERROR);
}

TEST_F(EpisodeHwTest, lifecycle_read_write_tracks_command)
{
  ASSERT_EQ(init_with(make_urdf(6)), CallbackReturn::SUCCESS);
  export_all();
  ASSERT_EQ(states_.size(), 12u);
  ASSERT_EQ(commands_.size(), 6u);
  ASSERT_EQ(hw_.on_configure(st_), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_.on_activate(st_), CallbackReturn::SUCCESS);

  // vertical posture == ROS zero; activation holds the current position
  for (int j = 1; j <= 6; ++j) {
    EXPECT_NEAR(state("joint" + std::to_string(j) + "/position"), 0.0, 1e-6);
  }
  ASSERT_EQ(cycle(), return_type::OK);

  command("joint1/position", 0.2);  // ~11.5 deg; mock moves 60 deg/s at ratio 1
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(cycle(), return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_NEAR(state("joint1/position"), 0.2, 0.01);
  EXPECT_NEAR(state("joint2/position"), 0.0, 1e-6);
  EXPECT_NEAR(state("joint1/velocity"), 0.0, 0.05);

  EXPECT_EQ(hw_.on_deactivate(st_), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_.on_cleanup(st_), CallbackReturn::SUCCESS);
}

TEST_F(EpisodeHwTest, write_outside_limits_is_error)
{
  ASSERT_EQ(init_with(make_urdf(6)), CallbackReturn::SUCCESS);
  export_all();
  ASSERT_EQ(hw_.on_configure(st_), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_.on_activate(st_), CallbackReturn::SUCCESS);
  command("joint2/position", 2.0);  // ~115 deg from vertical -> motor angle > 180
  EXPECT_EQ(cycle(), return_type::ERROR);
  EXPECT_EQ(hw_.on_cleanup(st_), CallbackReturn::SUCCESS);
}

TEST_F(EpisodeHwTest, nan_command_is_ignored)
{
  ASSERT_EQ(init_with(make_urdf(6)), CallbackReturn::SUCCESS);
  export_all();
  ASSERT_EQ(hw_.on_configure(st_), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_.on_activate(st_), CallbackReturn::SUCCESS);
  command("joint3/position", std::nan(""));
  EXPECT_EQ(cycle(), return_type::OK);
  EXPECT_EQ(hw_.on_cleanup(st_), CallbackReturn::SUCCESS);
}

}  // namespace
