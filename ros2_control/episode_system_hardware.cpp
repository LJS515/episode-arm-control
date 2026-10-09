// Copyright (c) 2026 Shikai Lou and Episode Arm Control contributors. MIT License.
//
// STATUS: compiled and tested in ROS2 Jazzy (WSL2) with the mock backend.
// Not run on hardware. See README.md.

#include "episode_system_hardware.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace episode_ros2_control
{
namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("EpisodeSystemHardware");
constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kMaxFrame = 1024 * 1024;

double param_or(const hardware_interface::HardwareInfo & info, const std::string & key, double dflt)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? dflt : std::stod(it->second);
}

std::string param_or(const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & dflt)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? dflt : it->second;
}

// "1,1,1,1,1,1" -> array
bool parse_six(const std::string & csv, JointArray & out)
{
  std::stringstream ss(csv);
  std::string tok;
  std::size_t i = 0;
  while (std::getline(ss, tok, ',')) {
    if (i >= kNumJoints) {return false;}
    out[i++] = std::stod(tok);
  }
  return i == kNumJoints;
}

// ---- tiny socket helpers (POSIX). Deadline-based, short-read safe. -------------
bool wait_fd(int fd, short events, int timeout_ms)
{
  pollfd p{fd, events, 0};
  int r;
  do {r = ::poll(&p, 1, timeout_ms);} while (r < 0 && errno == EINTR);
  return r > 0 && (p.revents & events);
}

bool send_all(int fd, const char * data, std::size_t n, int timeout_ms)
{
  std::size_t sent = 0;
  while (sent < n) {
    if (!wait_fd(fd, POLLOUT, timeout_ms)) {return false;}
    ssize_t k = ::send(fd, data + sent, n - sent, MSG_NOSIGNAL);
    if (k <= 0) {return false;}
    sent += static_cast<std::size_t>(k);
  }
  return true;
}

// recv_exact: loop until exactly n bytes arrived — a single recv() may return less.
bool recv_exact(int fd, char * buf, std::size_t n, int timeout_ms)
{
  std::size_t got = 0;
  while (got < n) {
    if (!wait_fd(fd, POLLIN, timeout_ms)) {return false;}
    ssize_t k = ::recv(fd, buf + got, n - got, 0);
    if (k <= 0) {return false;}  // 0 = peer closed (truncated frame)
    got += static_cast<std::size_t>(k);
  }
  return true;
}

// Parse a JSON array of 6 numbers / nulls, e.g. "[180.0, null, 83, 30, 110, 30]".
// Deliberately minimal: only what get_motor_angles returns.
bool parse_angle_array(const std::string & s, JointArray & out, ValidArray & valid)
{
  std::size_t i = s.find('[');
  if (i == std::string::npos) {return false;}
  ++i;
  for (std::size_t j = 0; j < kNumJoints; ++j) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {++i;}
    if (s.compare(i, 4, "null") == 0) {
      valid[j] = false;
      out[j] = 0.0;
      i += 4;
    } else {
      char * end = nullptr;
      out[j] = std::strtod(s.c_str() + i, &end);
      if (end == s.c_str() + i || !std::isfinite(out[j])) {return false;}
      valid[j] = true;
      i = static_cast<std::size_t>(end - s.c_str());
    }
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {++i;}
    char expect = (j + 1 == kNumJoints) ? ']' : ',';
    if (i >= s.size() || s[i] != expect) {return false;}
    ++i;
  }
  return true;
}
}  // namespace

// ============================================================================ MockBackend
MockBackend::MockBackend(double max_speed_deg_s)
: max_speed_(max_speed_deg_s), last_(std::chrono::steady_clock::now()) {}

bool MockBackend::connect() {return true;}

void MockBackend::step()
{
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_).count();
  last_ = now;
  if (estopped_) {return;}
  double max_step = max_speed_ * speed_ratio_ * dt;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    double e = goal_[i] - pos_[i];
    pos_[i] += std::clamp(e, -max_step, max_step);
  }
}

bool MockBackend::read_motor_deg(JointArray & out, ValidArray & valid)
{
  std::lock_guard<std::mutex> lk(mutex_);
  step();
  out = pos_;
  valid.fill(true);
  return true;
}

bool MockBackend::command_motor_deg(const JointArray & target, double speed_ratio)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (estopped_) {return false;}
  step();
  goal_ = target;
  speed_ratio_ = speed_ratio;
  return true;
}

bool MockBackend::emergency_stop()
{
  std::lock_guard<std::mutex> lk(mutex_);
  step();
  goal_ = pos_;
  estopped_ = true;
  return true;
}

// ============================================================================ VendorTcpBackend
VendorTcpBackend::VendorTcpBackend(std::string host, int port, int timeout_ms)
: host_(std::move(host)), port_(port), timeout_ms_(timeout_ms) {}

std::string VendorTcpBackend::describe() const
{
  return "official upper computer " + host_ + ":" + std::to_string(port_) + " (legacy JSON)";
}

bool VendorTcpBackend::connect()
{
  JointArray a{};
  ValidArray v{};
  return read_motor_deg(a, v);  // probe
}

bool VendorTcpBackend::transact(const std::string & req, std::string & reply)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * res = nullptr;
  if (::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res) {return false;}
  int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  bool ok = fd >= 0 && ::connect(fd, res->ai_addr, res->ai_addrlen) == 0;
  ::freeaddrinfo(res);
  if (ok) {
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    char hdr[8];
    std::uint64_t n = req.size();
    for (int i = 7; i >= 0; --i) {hdr[i] = static_cast<char>(n & 0xFF); n >>= 8;}  // big endian
    ok = send_all(fd, hdr, 8, timeout_ms_) && send_all(fd, req.data(), req.size(), timeout_ms_);
    if (ok) {ok = recv_exact(fd, hdr, 8, timeout_ms_);}
    if (ok) {
      std::uint64_t len = 0;
      for (int i = 0; i < 8; ++i) {len = (len << 8) | static_cast<unsigned char>(hdr[i]);}
      ok = len > 0 && len <= kMaxFrame;  // bound before allocating
      if (ok) {
        reply.assign(static_cast<std::size_t>(len), '\0');
        ok = recv_exact(fd, reply.data(), reply.size(), timeout_ms_);
      }
    }
  }
  if (fd >= 0) {::close(fd);}
  return ok;
}

bool VendorTcpBackend::read_motor_deg(JointArray & out, ValidArray & valid)
{
  std::string reply;
  if (!transact(R"({"action":"get_motor_angles","params":null})", reply)) {return false;}
  return parse_angle_array(reply, out, valid);
}

bool VendorTcpBackend::command_motor_deg(const JointArray & target, double speed_ratio)
{
  std::ostringstream ss;
  ss.precision(6);
  ss << std::fixed << R"({"action":"angle_mode","params":[[)";
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(target[i])) {return false;}
    ss << (i ? "," : "") << target[i];
  }
  ss << "]," << speed_ratio << "]}";
  std::string reply;
  // reply = estimated duration or -1; anything parseable as -1 is a rejection
  if (!transact(ss.str(), reply)) {return false;}
  return reply.find("-1") != 0;
}

bool VendorTcpBackend::emergency_stop()
{
  std::string reply;
  return transact(R"({"action":"emergency_stop","params":null})", reply);
}

// ============================================================================ SystemInterface
CallbackReturn EpisodeSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();
  if (info.joints.size() != kNumJoints) {
    RCLCPP_FATAL(get_logger(), "expected %zu joints, got %zu", kNumJoints, info.joints.size());
    return CallbackReturn::ERROR;
  }
  pos_state_names_.clear();
  vel_state_names_.clear();
  pos_cmd_names_.clear();
  for (const auto & j : info.joints) {
    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "joint %s: exactly one 'position' command interface required", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    bool has_pos = false;
    bool has_vel = false;
    for (const auto & si : j.state_interfaces) {
      has_pos = has_pos || si.name == hardware_interface::HW_IF_POSITION;
      has_vel = has_vel || si.name == hardware_interface::HW_IF_VELOCITY;
    }
    if (!has_pos || !has_vel) {
      RCLCPP_FATAL(get_logger(), "joint %s: 'position' and 'velocity' state interfaces required", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    pos_state_names_.push_back(j.name + "/" + hardware_interface::HW_IF_POSITION);
    vel_state_names_.push_back(j.name + "/" + hardware_interface::HW_IF_VELOCITY);
    pos_cmd_names_.push_back(j.name + "/" + hardware_interface::HW_IF_POSITION);
  }
  backend_name_ = param_or(info, "backend", std::string("mock"));
  auto csv = [&](const char * key, JointArray & dst) {
      auto it = info.hardware_parameters.find(key);
      return it == info.hardware_parameters.end() || parse_six(it->second, dst);
    };
  if (!csv("mapping_signs", signs_) || !csv("mapping_offsets_deg", offsets_deg_) ||
    !csv("joint_lower_deg", lower_deg_) || !csv("joint_upper_deg", upper_deg_))
  {
    RCLCPP_FATAL(get_logger(), "malformed 6-element parameter");
    return CallbackReturn::ERROR;
  }
  for (double sgn : signs_) {
    if (sgn != 1.0 && sgn != -1.0) {
      RCLCPP_FATAL(get_logger(), "mapping_signs must be +1/-1");
      return CallbackReturn::ERROR;
    }
  }
  soft_margin_deg_ = param_or(info, "soft_margin_deg", soft_margin_deg_);
  stale_after_s_ = param_or(info, "stale_after_s", stale_after_s_);
  poll_rate_hz_ = param_or(info, "poll_rate_hz", poll_rate_hz_);
  command_deadband_deg_ = param_or(info, "command_deadband_deg", command_deadband_deg_);
  min_command_interval_s_ = param_or(info, "min_command_interval_s", min_command_interval_s_);
  speed_ratio_ = std::clamp(param_or(info, "speed_ratio", speed_ratio_), 0.01, 1.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn EpisodeSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (backend_name_ == "mock") {
    backend_ = std::make_unique<MockBackend>();
  } else if (backend_name_ == "vendor_tcp") {
    backend_ = std::make_unique<VendorTcpBackend>(
      param_or(get_hardware_info(), "host", std::string("127.0.0.1")),
      static_cast<int>(param_or(get_hardware_info(), "port", 12345.0)),
      static_cast<int>(param_or(get_hardware_info(), "timeout_ms", 300.0)));
  } else {
    // "vendor_core" (official compiled control core, route B) would go here.
    RCLCPP_FATAL(get_logger(), "unknown backend '%s'", backend_name_.c_str());
    return CallbackReturn::ERROR;
  }
  if (!backend_->connect()) {
    RCLCPP_ERROR(get_logger(), "backend %s: connect/probe failed", backend_->describe().c_str());
    return CallbackReturn::ERROR;
  }
  polling_ = true;
  poll_thread_ = std::thread(&EpisodeSystemHardware::poll_loop, this);
  RCLCPP_INFO(get_logger(), "configured: %s", backend_->describe().c_str());
  return CallbackReturn::SUCCESS;
}

void EpisodeSystemHardware::poll_loop()
{
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, poll_rate_hz_));
  while (polling_) {
    auto t0 = std::chrono::steady_clock::now();
    JointArray m{};
    ValidArray v{};
    bool ok = backend_->read_motor_deg(m, v);
    {
      std::lock_guard<std::mutex> lk(sample_mutex_);
      if (ok && std::all_of(v.begin(), v.end(), [](bool b) {return b;})) {
        last_motor_deg_ = m;
        last_valid_ = v;
        last_sample_time_ = t0;
        have_sample_ = true;
        consecutive_failures_ = 0;
      } else {
        ++consecutive_failures_;
      }
    }
    std::this_thread::sleep_until(t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
  }
}

CallbackReturn EpisodeSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // Wait briefly for a fresh sample; hold the current position as command
  // so the controller does not jump to a stale/NaN target.
  for (int i = 0; i < 50; ++i) {
    {
      std::lock_guard<std::mutex> lk(sample_mutex_);
      if (have_sample_) {
        for (std::size_t j = 0; j < kNumJoints; ++j) {
          const double q = motor_to_model_rad(j, last_motor_deg_[j]);
          set_state(pos_state_names_[j], q);
          set_state(vel_state_names_[j], 0.0);
          set_command(pos_cmd_names_[j], q);  // hold current position: no jump on activation
          prev_model_rad_[j] = q;
        }
        have_prev_ = true;
        have_sent_ = false;
        RCLCPP_INFO(get_logger(), "activated");
        return CallbackReturn::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_ERROR(get_logger(), "no valid feedback within 1 s; refusing to activate");
  return CallbackReturn::ERROR;
}

CallbackReturn EpisodeSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // The official API has no documented soft stop (evidence B); deactivation
  // while moving therefore escalates to emergency_stop.
  JointArray m{};
  ValidArray v{};
  if (backend_ && backend_->read_motor_deg(m, v) && have_sent_) {
    double max_err = 0.0;
    for (std::size_t j = 0; j < kNumJoints; ++j) {max_err = std::max(max_err, std::abs(m[j] - last_sent_motor_deg_[j]));}
    if (max_err > 1.0) {
      RCLCPP_WARN(get_logger(), "deactivated while moving -> emergency_stop");
      backend_->emergency_stop();
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn EpisodeSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  polling_ = false;
  if (poll_thread_.joinable()) {poll_thread_.join();}
  if (backend_) {backend_->disconnect();}
  backend_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn EpisodeSystemHardware::on_shutdown(const rclcpp_lifecycle::State & s)
{
  return on_cleanup(s);
}

CallbackReturn EpisodeSystemHardware::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "on_error -> emergency_stop");
  if (backend_) {backend_->emergency_stop();}
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EpisodeSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lk(sample_mutex_);
  if (!have_sample_) {return hardware_interface::return_type::ERROR;}
  double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_sample_time_).count();
  if (age > stale_after_s_ || consecutive_failures_ >= 3) {
    // Stale feedback must not be reported as current state.
    RCLCPP_ERROR_THROTTLE(get_logger(), throttle_clock_, 1000,
      "feedback stale (age %.3f s, failures %d)", age, consecutive_failures_);
    return hardware_interface::return_type::ERROR;
  }
  const double dt = std::max(period.seconds(), 1e-6);
  for (std::size_t j = 0; j < kNumJoints; ++j) {
    const double q = motor_to_model_rad(j, last_motor_deg_[j]);
    set_state(pos_state_names_[j], q);
    set_state(vel_state_names_[j], have_prev_ ? (q - prev_model_rad_[j]) / dt : 0.0);
    prev_model_rad_[j] = q;
  }
  have_prev_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EpisodeSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  JointArray target{};
  for (std::size_t j = 0; j < kNumJoints; ++j) {
    const double cmd = get_command<double>(pos_cmd_names_[j]);
    if (!std::isfinite(cmd)) {return hardware_interface::return_type::OK;}  // no command yet
    target[j] = model_rad_to_motor(j, cmd);
  }
  if (!within_limits(target, soft_margin_deg_)) {
    RCLCPP_ERROR(get_logger(), "command outside joint limits -> ERROR");
    return hardware_interface::return_type::ERROR;
  }
  // Rate limiting / deadband: the official API is point-to-point (each command
  // plans its own profile), not a streaming setpoint interface. Sending every
  // 20 ms interpolated JTC setpoint would flood the TCP server and produce
  // stop-and-go motion. dynamic_move may be the streaming path (evidence E).
  auto now = std::chrono::steady_clock::now();
  if (have_sent_) {
    double max_delta = 0.0;
    for (std::size_t j = 0; j < kNumJoints; ++j) {max_delta = std::max(max_delta, std::abs(target[j] - last_sent_motor_deg_[j]));}
    double since = std::chrono::duration<double>(now - last_send_time_).count();
    if (max_delta < command_deadband_deg_ || since < min_command_interval_s_) {
      return hardware_interface::return_type::OK;
    }
  }
  if (!backend_->command_motor_deg(target, speed_ratio_)) {
    RCLCPP_ERROR(get_logger(), "backend rejected command");
    return hardware_interface::return_type::ERROR;
  }
  last_sent_motor_deg_ = target;
  last_send_time_ = now;
  have_sent_ = true;
  return hardware_interface::return_type::OK;
}

double EpisodeSystemHardware::motor_to_model_rad(std::size_t i, double motor_deg) const
{
  return signs_[i] * (motor_deg - offsets_deg_[i]) * kPi / 180.0;
}

double EpisodeSystemHardware::model_rad_to_motor(std::size_t i, double model_rad) const
{
  return offsets_deg_[i] + (model_rad * 180.0 / kPi) / signs_[i];
}

bool EpisodeSystemHardware::within_limits(const JointArray & m, double margin) const
{
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (m[i] < lower_deg_[i] + margin || m[i] > upper_deg_[i] - margin) {return false;}
  }
  return true;
}

}  // namespace episode_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(episode_ros2_control::EpisodeSystemHardware, hardware_interface::SystemInterface)
