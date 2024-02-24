#include <chrono> // IWYU pragma: keep
#include <functional> // IWYU pragma: keep
#include <general_interfaces/msg/detail/arm_pose__struct.hpp>
#include <general_interfaces/msg/detail/reference_positions__struct.hpp>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "std_msgs/msg/string.hpp" // IWYU pragma: keep
#include "general_interfaces/msg/arm_pose.hpp" // IWYU pragma: keep
#include "control_msgs/msg/joint_trajectory_controller_state.hpp" // IWYU pragma: keep

using namespace std::chrono_literals;

//4096 counts

class IKHardwareInterface : public rclcpp::Node
{
  public:
    IKHardwareInterface() : Node("HardwareInterface") {

        // replace this with the joint_states publisher type
        upstream_publisher_ = this->create_publisher<general_interfaces::msg::ArmPose>("goal_states", 10);

        upstream_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
                "/uorover_arm_controller/controller_state",
                10,
                std::bind(&IKHardwareInterface::ik_callback, this, std::placeholders::_1));

        downstream_subscription_ = this->create_subscription<general_interfaces::msg::ArmPose>(
                "arm_feedback",
                10,
                std::bind(&IKHardwareInterface::arm_callback, this, std::placeholders::_1));
            
        downstream_publisher_  = this->create_publisher<general_interfaces::msg::ArmPose>("goal_states", 10);
    }


    // getters and setters for private reference positions attribute
    std::vector<double> getReferencePositions() const {
        return referencePositions_;
    }

    void setReferencePositions(const std::vector<double> &referencePositions){
        referencePositions_ = referencePositions;
    } 

    std::vector<double> getArmPositions() const {
        return armPositions_;
    }

    void setArmPositions(const std::vector<double> &armPositions){
        armPositions_ = armPositions;
    } 

  private:
    // on controller state topic callback, update reference positions attribute
    void ik_callback(const control_msgs::msg::JointTrajectoryControllerState msg){
        this->setReferencePositions(msg.reference.positions);
    }

    void arm_callback(const general_interfaces::msg::ArmPose msg){
        this->setArmPositions(msg.positions);
    }

    // publish joint states to arm hardware on a timer
    void publish(){
        general_interfaces::msg::ArmPose message;;
        message.positions = this->getReferencePositions();
        downstream_publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<general_interfaces::msg::ArmPose>::SharedPtr upstream_publisher_;
    rclcpp::Publisher<general_interfaces::msg::ArmPose>::SharedPtr downstream_publisher_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr upstream_subscription_;
    rclcpp::Subscription<general_interfaces::msg::ArmPose>::SharedPtr downstream_subscription_;
    std::vector<double> referencePositions_;
    std::vector<double> armPositions_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKHardwareInterface>());
  rclcpp::shutdown();
  return 0;
}
