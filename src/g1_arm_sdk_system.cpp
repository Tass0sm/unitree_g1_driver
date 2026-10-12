#include "unitree_g1_driver/g1_arm_sdk_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "unitree_g1_driver/motor_crc_hg.hpp"

namespace unitree_g1_driver
{
namespace
{
// Motor indices of the 29-DoF G1 in LowCmd/LowState (unitree_ros2 example/src/include/g1/g1.hpp, G1Arm7JointIndex).
const std::map<std::string, int> kMotorIndex = {
  {"left_hip_pitch_joint", 0},        {"left_hip_roll_joint", 1},
  {"left_hip_yaw_joint", 2},          {"left_knee_joint", 3},
  {"left_ankle_pitch_joint", 4},      {"left_ankle_roll_joint", 5},
  {"right_hip_pitch_joint", 6},       {"right_hip_roll_joint", 7},
  {"right_hip_yaw_joint", 8},         {"right_knee_joint", 9},
  {"right_ankle_pitch_joint", 10},    {"right_ankle_roll_joint", 11},
  {"waist_yaw_joint", 12},            {"waist_roll_joint", 13},
  {"waist_pitch_joint", 14},          {"left_shoulder_pitch_joint", 15},
  {"left_shoulder_roll_joint", 16},   {"left_shoulder_yaw_joint", 17},
  {"left_elbow_joint", 18},           {"left_wrist_roll_joint", 19},
  {"left_wrist_pitch_joint", 20},     {"left_wrist_yaw_joint", 21},
  {"right_shoulder_pitch_joint", 22}, {"right_shoulder_roll_joint", 23},
  {"right_shoulder_yaw_joint", 24},   {"right_elbow_joint", 25},
  {"right_wrist_roll_joint", 26},     {"right_wrist_pitch_joint", 27},
  {"right_wrist_yaw_joint", 28},
};
// Joints the arm SDK accepts commands for (waist and arms).
constexpr int kFirstUpperBodyIndex = 12;
// The unused motor slot whose `q` carries the arm SDK blend weight.
constexpr int kDefaultWeightIndex = 29;

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

std::string param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & name,
  const std::string & default_value)
{
  const auto it = params.find(name);
  return it == params.end() ? default_value : it->second;
}
}  // namespace

G1ArmSdkSystem::~G1ArmSdkSystem()
{
  stop_io();
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    const auto & hw = info_.hardware_parameters;
    state_topic_ = param_or(hw, "state_topic", "lowstate");
    command_topic_ = param_or(hw, "command_topic", "arm_sdk");
    state_timeout_ = std::stod(param_or(hw, "state_timeout", "0.1"));
    activation_timeout_ = std::stod(param_or(hw, "activation_timeout", "3.0"));
    weight_ramp_time_ = std::stod(param_or(hw, "weight_ramp_time", "2.0"));
    max_joint_velocity_ = std::stod(param_or(hw, "max_joint_velocity", "1.0"));
    weight_index_ = std::stoi(param_or(hw, "weight_index", std::to_string(kDefaultWeightIndex)));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (const auto & info : info_.joints)
  {
    Joint joint;
    joint.name = info.name;

    const auto index = kMotorIndex.find(info.name);
    if (index == kMotorIndex.end())
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' is not a G1 29-DoF motor.", info.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint.motor_index = index->second;

    if (info.state_interfaces.empty() ||
        info.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs 'position' as its first state interface.",
                   info.name.c_str());
      return CallbackReturn::ERROR;
    }

    joint.commanded = false;
    for (const auto & cmd : info.command_interfaces)
    {
      if (cmd.name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s': only 'position' commands are supported, got '%s'.",
                     info.name.c_str(), cmd.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint.commanded = true;
    }

    joint.kp = 0.0;
    joint.kd = 0.0;
    if (joint.commanded)
    {
      if (joint.motor_index < kFirstUpperBodyIndex)
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s' is a leg joint; the arm SDK cannot command it.",
                     info.name.c_str());
        return CallbackReturn::ERROR;
      }
      if (!info.parameters.count("kp") || !info.parameters.count("kd"))
      {
        RCLCPP_ERROR(get_logger(), "Commanded joint '%s' needs 'kp' and 'kd' parameters.",
                     info.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint.kp = std::stod(info.parameters.at("kp"));
      joint.kd = std::stod(info.parameters.at("kd"));
    }

    joint.min_position = -std::numeric_limits<double>::infinity();
    joint.max_position = std::numeric_limits<double>::infinity();
    const auto limits = info_.limits.find(info.name);
    if (limits != info_.limits.end() && limits->second.has_position_limits)
    {
      joint.min_position = limits->second.min_position;
      joint.max_position = limits->second.max_position;
    }

    joints_.push_back(joint);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & joint : joints_)
  {
    joint.position_state = get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    const auto velocity = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto effort = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint.velocity_state = has_state(velocity) ? get_state_interface_handle(velocity) : nullptr;
    joint.effort_state = has_state(effort) ? get_state_interface_handle(effort) : nullptr;
    joint.position_command =
      joint.commanded ? get_command_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION)
                      : nullptr;
  }

  try
  {
    start_io();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to start DDS I/O: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_io();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!wait_for_state(activation_timeout_))
  {
    RCLCPP_ERROR(get_logger(), "No fresh message on '%s' within %.1f s; not activating.",
                 state_topic_.c_str(), activation_timeout_);
    return CallbackReturn::ERROR;
  }

  // Not concurrent with read()/write(), so reading the buffer here is safe.
  const auto state = *state_buffer_.readFromRT();

  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cmd_msg_ = LowCmd();
  cmd_msg_.mode_pr = 0;
  cmd_msg_.mode_machine = state->mode_machine;
  for (auto & joint : joints_)
  {
    if (!joint.commanded)
    {
      continue;
    }
    // Start by holding the measured pose, both in what we send and in the command interface.
    joint.last_sent = state->motor_state[joint.motor_index].q;
    std::ignore = set_command(joint.position_command, joint.last_sent, true);

    auto & motor = cmd_msg_.motor_cmd[joint.motor_index];
    motor.mode = 1;
    motor.dq = 0.0F;
    motor.tau = 0.0F;
    motor.kp = static_cast<float>(joint.kp);
    motor.kd = static_cast<float>(joint.kd);
  }

  weight_ = 0.0;
  active_ = true;
  RCLCPP_INFO(get_logger(), "Activated (mode_machine %u); ramping arm SDK weight to 1 over %.1f s.",
              state->mode_machine, weight_ramp_time_);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_arms();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_logger(), "Error: handing the arms back to Unitree's controller.");
  release_arms();
  stop_io();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn G1ArmSdkSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_arms();
  stop_io();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type G1ArmSdkSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto * state = state_buffer_.readFromRT();
  if (state == nullptr || !*state)
  {
    return hardware_interface::return_type::OK;  // nothing received yet (only possible while inactive)
  }

  const double age = 1e-9 * static_cast<double>(steady_now_ns() - last_state_ns_.load());
  if (age > state_timeout_)
  {
    if (active_)
    {
      RCLCPP_ERROR(get_logger(), "'%s' is %.3f s old (timeout %.3f s).", state_topic_.c_str(), age,
                   state_timeout_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "'%s' is stale (%.1f s old).",
                         state_topic_.c_str(), age);
  }

  for (const auto & joint : joints_)
  {
    const auto & motor = (*state)->motor_state[joint.motor_index];
    std::ignore = set_state(joint.position_state, static_cast<double>(motor.q), false);
    if (joint.velocity_state)
    {
      std::ignore = set_state(joint.velocity_state, static_cast<double>(motor.dq), false);
    }
    if (joint.effort_state)
    {
      std::ignore = set_state(joint.effort_state, static_cast<double>(motor.tau_est), false);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1ArmSdkSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::unique_lock<std::mutex> lock(cmd_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !active_)
  {
    return hardware_interface::return_type::OK;  // a release is in progress, or not active
  }

  const double dt = std::clamp(period.seconds(), 0.0, 0.05);
  weight_ = weight_ramp_time_ > 0.0 ? std::min(1.0, weight_ + dt / weight_ramp_time_) : 1.0;

  const double max_step = max_joint_velocity_ * dt;
  for (auto & joint : joints_)
  {
    if (!joint.commanded)
    {
      continue;
    }
    double target = joint.last_sent;
    double command;
    if (get_command(joint.position_command, command, false) && std::isfinite(command))
    {
      target = std::clamp(command, joint.min_position, joint.max_position);
    }
    joint.last_sent += std::clamp(target - joint.last_sent, -max_step, max_step);
  }

  fill_and_publish(weight_);
  return hardware_interface::return_type::OK;
}

void G1ArmSdkSystem::fill_and_publish(double weight)
{
  for (const auto & joint : joints_)
  {
    if (joint.commanded)
    {
      cmd_msg_.motor_cmd[joint.motor_index].q = static_cast<float>(joint.last_sent);
    }
  }
  cmd_msg_.motor_cmd[weight_index_].q = static_cast<float>(weight);
  fill_crc(cmd_msg_);
  cmd_pub_->publish(cmd_msg_);
}

void G1ArmSdkSystem::release_arms()
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  if (!active_)
  {
    return;
  }
  active_ = false;
  if (!io_node_ || !cmd_pub_ || !rclcpp::ok(io_node_->get_node_base_interface()->get_context()))
  {
    RCLCPP_ERROR(get_logger(), "Cannot publish the arm SDK release; arms stay on the last command.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Ramping arm SDK weight from %.2f to 0.", weight_);
  constexpr double dt = 0.01;
  const int steps = std::max(1, static_cast<int>(std::ceil(weight_ramp_time_ / dt)));
  const double start = weight_;
  for (int i = 1; i <= steps; ++i)
  {
    weight_ = start * (1.0 - static_cast<double>(i) / steps);
    fill_and_publish(weight_);
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }
  weight_ = 0.0;
}

bool G1ArmSdkSystem::wait_for_state(double timeout_s) const
{
  const auto deadline = steady_now_ns() + static_cast<int64_t>(timeout_s * 1e9);
  while (steady_now_ns() < deadline)
  {
    const double age = 1e-9 * static_cast<double>(steady_now_ns() - last_state_ns_.load());
    if (last_state_ns_.load() != 0 && age < state_timeout_)
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

void G1ArmSdkSystem::start_io()
{
  stop_io();

  std::string node_name = info_.name + "_io";
  std::replace_if(node_name.begin(), node_name.end(), [](unsigned char c) { return !std::isalnum(c); }, '_');
  io_node_ = std::make_shared<rclcpp::Node>(
    node_name, rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  // Unitree publishes lowstate best effort at 500 Hz; only the latest sample matters.
  state_sub_ = io_node_->create_subscription<LowState>(
    state_topic_, rclcpp::SensorDataQoS().keep_last(1), [this](LowState::UniquePtr msg) {
      state_buffer_.writeFromNonRT(std::shared_ptr<LowState>(std::move(msg)));
      last_state_ns_.store(steady_now_ns());
    });
  cmd_pub_ = io_node_->create_publisher<LowCmd>(command_topic_, rclcpp::QoS(10));

  // Release the arms before ROS shuts down (Ctrl+C), while publishing still works.
  shutdown_callback_ = io_node_->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    [this]() { release_arms(); });

  io_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  io_executor_->add_node(io_node_);
  io_thread_ = std::thread([this]() { io_executor_->spin(); });

  RCLCPP_INFO(get_logger(), "Subscribed to '%s', publishing on '%s'.",
              state_sub_->get_topic_name(), cmd_pub_->get_topic_name());
}

void G1ArmSdkSystem::stop_io()
{
  if (io_executor_)
  {
    io_executor_->cancel();
  }
  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
  if (io_node_)
  {
    io_node_->get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
      shutdown_callback_);
  }
  io_executor_.reset();
  state_sub_.reset();
  cmd_pub_.reset();
  io_node_.reset();
  last_state_ns_.store(0);
}

}  // namespace unitree_g1_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_g1_driver::G1ArmSdkSystem, hardware_interface::SystemInterface)
