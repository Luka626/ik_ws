#ifndef UOROVER_IK__RMM_HARDWARE_HPP_
#define UOROVER_IK__RMM_HARDWARE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "general_interfaces/msg/arm_pose.hpp" // IWYU pragma: keep
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <general_interfaces/msg/detail/arm_pose__struct.hpp>
#include <unordered_map>

namespace uorover_ik {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC ArmSystem
    : public hardware_interface::SystemInterface {
public:
  // initialize function
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  // TODO: implement remaining state transitions
  // priority ordered:
  //  - on_error
  //  - on_configure
  //  - on_shutdown
  //  - on_cleanup
  //  - on_activate
  //  - on_deactivate

  // interface export
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // hw read/write
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/) override;

protected:
  // necessary for topic-based control
  // eventually, we should merge this code with router and send serial commands
  // from here
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<general_interfaces::msg::ArmPose>::SharedPtr
      arm_feedback_subscriber_;
  rclcpp::Publisher<general_interfaces::msg::ArmPose>::SharedPtr
      goal_state_publisher_;

  // TODO: if a command message is within +/- threshold_ of latest_pose_, dont
  // send it
  general_interfaces::msg::ArmPose latest_pose_;
  double threshold_;

  void arm_callback(const general_interfaces::msg::ArmPose &msg);

  // interfaces
  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  enum Joint { TOWER, SHOULDER, ELBOW, WRIST, LAST };

  // TODO: KILL WHEN ABSOLUTE ENCODERS COME IN HALLELUJAH
  // 4096 counts
  struct LinearActuator {
    int id;
    float lMin;
    float lMax;
    float thetaMin;
    float thetaMax;
    float slope;
  };
  LinearActuator LA1_;
  LinearActuator LA2_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
      {"position", {}}, {"velocity", {}}};

}; // class ArmSystem

} // namespace uorover_ik

#endif // uorover_ik__rmm_hardware_hpp_
