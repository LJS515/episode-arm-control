// Copyright (c) 2026 Shikai Lou and Episode Arm Control contributors. MIT License.
//
// STATUS: compiled and tested in ROS2 Jazzy (WSL2 Ubuntu 24.04, ros2_control
// 4.48) with the MOCK backend only. Not run on hardware. Uses the current
// hardware_interface API: on_init(HardwareComponentInterfaceParams), interfaces
// created by the framework from the URDF, values accessed through
// set_state()/get_command() (the vector-returning export_*_interfaces() and
// on_init(HardwareInfo) are deprecated in this release).
//
// What it is: a ros2_control SystemInterface for the Episode 1 arm that
// talks to one of two backends:
//   * MockBackend       — in-process first-order simulation (Synthetic / Mock Data)
//   * VendorTcpBackend  — the OFFICIAL Windows upper computer over TCP/JSON
//                         (8-byte big-endian length + UTF-8 JSON), route A.
// A third backend wrapping the official compiled control core (route B) is a
// placeholder: its C API is not public.
//
// What it is NOT: it does not talk CAN, does not implement FOC, does not
// replace the official control core. Those remain vendor components.

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace episode_ros2_control
{

constexpr std::size_t kNumJoints = 6;
using JointArray = std::array<double, kNumJoints>;
using ValidArray = std::array<bool, kNumJoints>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// --------------------------------------------------------------------------
// Backend abstraction. All angles are MOTOR degrees (official coordinate).
// --------------------------------------------------------------------------
class EpisodeBackend
{
public:
  virtual ~EpisodeBackend() = default;
  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  // Returns false when nothing could be read; per-joint validity in `valid`
  // (the official reply can contain null for joints without CAN contact).
  virtual bool read_motor_deg(JointArray & out, ValidArray & valid) = 0;
  // Point-to-point target. Returns false if the backend rejected it.
  virtual bool command_motor_deg(const JointArray & target, double speed_ratio) = 0;
  virtual bool emergency_stop() = 0;
  virtual std::string describe() const = 0;
};

class MockBackend : public EpisodeBackend
{
public:
  explicit MockBackend(double max_speed_deg_s = 60.0);
  bool connect() override;
  void disconnect() override {}
  bool read_motor_deg(JointArray & out, ValidArray & valid) override;
  bool command_motor_deg(const JointArray & target, double speed_ratio) override;
  bool emergency_stop() override;
  std::string describe() const override { return "mock (Synthetic / Mock Data)"; }

private:
  void step();
  std::mutex mutex_;
  JointArray pos_{{180.0, 90.0, 83.0, 30.0, 110.0, 30.0}};
  JointArray goal_{{180.0, 90.0, 83.0, 30.0, 110.0, 30.0}};
  double max_speed_;
  double speed_ratio_{1.0};
  bool estopped_{false};
  std::chrono::steady_clock::time_point last_;
};

// Minimal blocking TCP client for the official upper computer. One request in
// flight at a time (guarded by io_mutex_). Uses a per-request connection,
// matching the historical official client.
class VendorTcpBackend : public EpisodeBackend
{
public:
  VendorTcpBackend(std::string host, int port, int timeout_ms);
  bool connect() override;
  void disconnect() override {}
  bool read_motor_deg(JointArray & out, ValidArray & valid) override;
  bool command_motor_deg(const JointArray & target, double speed_ratio) override;
  bool emergency_stop() override;
  std::string describe() const override;

private:
  bool transact(const std::string & json_request, std::string & json_reply);
  std::string host_;
  int port_;
  int timeout_ms_;
  std::mutex io_mutex_;
};

// --------------------------------------------------------------------------
// SystemInterface
// --------------------------------------------------------------------------
class EpisodeSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EpisodeSystemHardware)

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // feedback thread: decouples the (slow, blocking) TCP round trip from the
  // controller_manager's real-time-ish update loop
  void poll_loop();

  double motor_to_model_rad(std::size_t i, double motor_deg) const;
  double model_rad_to_motor(std::size_t i, double model_rad) const;
  bool within_limits(const JointArray & motor_deg, double margin_deg) const;

  std::unique_ptr<EpisodeBackend> backend_;

  // parameters (from <hardware><param> in the URDF/xacro)
  std::string backend_name_{"mock"};
  JointArray signs_{{1, 1, 1, 1, 1, 1}};              // evidence B: official node uses motor - offset
  JointArray offsets_deg_{{180, 90, 83, 30, 110, 30}};  // evidence B
  JointArray lower_deg_{{0, 0, 0, 0, 0, 0}};
  JointArray upper_deg_{{340, 180, 163, 335, 220, 335}};
  double soft_margin_deg_{2.0};
  double stale_after_s_{0.5};
  double poll_rate_hz_{20.0};
  double command_deadband_deg_{0.2};
  double min_command_interval_s_{0.1};
  double speed_ratio_{0.3};

  // full interface names, e.g. "joint1/position" (model radians)
  std::vector<std::string> pos_state_names_;
  std::vector<std::string> vel_state_names_;
  std::vector<std::string> pos_cmd_names_;
  JointArray prev_model_rad_{};
  bool have_prev_{false};

  // shared with the poll thread
  std::mutex sample_mutex_;
  JointArray last_motor_deg_{};
  ValidArray last_valid_{};
  std::chrono::steady_clock::time_point last_sample_time_{};
  bool have_sample_{false};
  int consecutive_failures_{0};

  std::thread poll_thread_;
  std::atomic<bool> polling_{false};

  JointArray last_sent_motor_deg_{};
  bool have_sent_{false};
  std::chrono::steady_clock::time_point last_send_time_{};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};

}  // namespace episode_ros2_control
