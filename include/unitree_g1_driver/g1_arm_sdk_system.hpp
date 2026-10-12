#ifndef UNITREE_G1_DRIVER__G1_ARM_SDK_SYSTEM_HPP_
#define UNITREE_G1_DRIVER__G1_ARM_SDK_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace unitree_g1_driver
{

/**
 * ros2_control system for the upper body of a Unitree G1 (29 DoF) through Unitree's arm SDK.
 *
 * State comes from `rt/lowstate` for every joint in the <ros2_control> tag. Joints with a
 * `position` command interface (arms and waist only) are sent on `rt/arm_sdk`, which Unitree's
 * locomotion controller blends with its own commands using a weight that this component ramps
 * up on activation and down on deactivation/error. Upper-body joints whose command interface no
 * controller claims hold the position measured at activation.
 *
 * DDS I/O runs on a private node and executor thread, independent of the controller manager.
 */
class G1ArmSdkSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(G1ArmSdkSystem)

  ~G1ArmSdkSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using LowState = unitree_hg::msg::LowState;
  using LowCmd = unitree_hg::msg::LowCmd;

  struct Joint
  {
    std::string name;
    int motor_index;
    bool commanded;
    double kp;
    double kd;
    double min_position;
    double max_position;

    hardware_interface::StateInterface::SharedPtr position_state;
    hardware_interface::StateInterface::SharedPtr velocity_state;
    hardware_interface::StateInterface::SharedPtr effort_state;
    hardware_interface::CommandInterface::SharedPtr position_command;

    double last_sent = 0.0;  // last position actually sent on arm_sdk (rate limited)
  };

  void start_io();
  void stop_io();
  bool wait_for_state(double timeout_s) const;
  // Blocking: publishes the held pose while ramping the blend weight to zero.
  void release_arms();
  void fill_and_publish(double weight);

  // Parameters (from <hardware><param> in the URDF)
  std::string state_topic_;
  std::string command_topic_;
  double state_timeout_;
  double activation_timeout_;
  double weight_ramp_time_;
  double max_joint_velocity_;
  int weight_index_;

  std::vector<Joint> joints_;

  // DDS I/O
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr io_executor_;
  std::thread io_thread_;
  rclcpp::Subscription<LowState>::SharedPtr state_sub_;
  rclcpp::Publisher<LowCmd>::SharedPtr cmd_pub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<LowState>> state_buffer_;
  std::atomic<int64_t> last_state_ns_{0};  // steady clock

  rclcpp::PreShutdownCallbackHandle shutdown_callback_;

  // Command path; cmd_mutex_ keeps write() (try_lock) and release_arms() from interleaving.
  std::mutex cmd_mutex_;
  LowCmd cmd_msg_;
  double weight_ = 0.0;
  bool active_ = false;
};

}  // namespace unitree_g1_driver

#endif  // UNITREE_G1_DRIVER__G1_ARM_SDK_SYSTEM_HPP_
