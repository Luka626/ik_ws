#include <chrono> // IWYU pragma: keep
#include <functional> // IWYU pragma: keep
#include <memory>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "std_msgs/msg/string.hpp" // IWYU pragma: keep
#include "general_interfaces/msg/arm_pose.hpp" // IWYU pragma: keep
#include "control_msgs/msg/joint_trajectory_controller_state.hpp" // IWYU pragma: keep
#include "sensor_msgs/msg/joint_state.hpp" // IWYU pragma: keep

using namespace std::chrono_literals;

enum Joint{
    TOWER,
    SHOULDER,
    ELBOW,
    WRIST,
    LAST
};

//4096 counts
struct LinearActuator{
    int id;
    float lMin;
    float lMax;
    float thetaMin;
    float thetaMax;
    float slope;
};

double round(const double& num){return floor(num + 0.5);}

class IKHardwareInterface : public rclcpp::Node
{
  public:
    IKHardwareInterface() : Node("HardwareInterface") {

        testing = false;

        // replace this with the joint_states publisher type
        upstream_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        upstream_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
                "/uorover_arm_controller/controller_state",
                10,
                std::bind(&IKHardwareInterface::ik_callback, this, std::placeholders::_1));
        downstream_publisher_  = this->create_publisher<general_interfaces::msg::ArmPose>("goal_states", 10);

        downstream_subscription_ = this->create_subscription<general_interfaces::msg::ArmPose>(
                "arm_feedback",
                10,
                std::bind(&IKHardwareInterface::arm_callback, this, std::placeholders::_1));
        

        LA1_.id        = 1;
        LA1_.lMax      = 0.00;
        LA1_.lMin      = 6.00;
        LA1_.thetaMax  = 77.00 * 0.0174533;
        LA1_.thetaMin  = -4.00 * 0.0174533;
        LA1_.slope     = 1.0 * (LA1_.lMax - LA1_.lMin) / (LA1_.thetaMax - LA1_.thetaMin);

        LA2_.id        = 2;
        LA2_.lMax      = 0.00;
        LA2_.lMin      = 6.00;
        LA2_.thetaMax  = 100.00 * 0.0174533;
        LA2_.thetaMin  = 0.00 * 0.0175433;
        LA2_.slope     = 1.0 * (LA2_.lMax - LA2_.lMin) / (LA2_.thetaMax - LA2_.thetaMin);

        referencePositions_.resize(4);
        armPositions_.resize(4);

    }

    // getters and setters for private reference positions attribute
    std::vector<double> getReferencePositions() const {
        return referencePositions_;
    }

    void setReferencePositions(const std::vector<double> &referencePositions){
        for (int jnt=TOWER; jnt<LAST; jnt++){
            switch (jnt){
                case TOWER:
                    referencePositions_[jnt] = referencePositions[jnt];
                    break;
                case SHOULDER:
                    referencePositions_[jnt] = LA1_.lMin + LA1_.slope * (referencePositions[jnt] - LA1_.thetaMin);
                    break;
                case ELBOW:
                    referencePositions_[jnt] = LA2_.lMax - LA2_.slope * (referencePositions[jnt] - LA2_.thetaMin);
                    break;
                case WRIST:
                    referencePositions_[jnt] = referencePositions[jnt];
                    break;
            }

        }
    }

    std::vector<double> getArmPositions() const {
        return armPositions_;
    }

    void setArmPositions(const std::vector<double> &armPositions){
        for (int jnt=TOWER; jnt<LAST; jnt++){
            switch (jnt){
                case TOWER:
                    armPositions_[jnt] = armPositions[jnt];
                    break;
                case SHOULDER:
                    armPositions_[jnt] = LA1_.thetaMin + (1/LA1_.slope) * (armPositions[jnt] - LA1_.lMin);
                    break;
                case ELBOW:
                    armPositions_[jnt] = LA2_.thetaMax - (1/LA2_.slope) * (armPositions[jnt] - LA2_.lMin);
                    break;
                case WRIST:
                    armPositions_[jnt] = armPositions[jnt];
                    break;
            }
        }
    } 

  private:
    // on controller state topic callback, update reference positions attribute
    void ik_callback(const control_msgs::msg::JointTrajectoryControllerState &msg){
        auto message = general_interfaces::msg::ArmPose();

        this->setReferencePositions(msg.reference.positions);
        message.positions = this->getReferencePositions();

        downstream_publisher_->publish(message);
    }

    void arm_callback(const general_interfaces::msg::ArmPose &msg){
        auto message = sensor_msgs::msg::JointState();
        std::vector<std::string> names = {"q1", "q2", "q3", "q4"};
        message.name = names;

        if (!testing){
            this->setArmPositions(msg.positions);
        } else{
            armPositions_ = referencePositions_;
        }

        message.position = this->getArmPositions();

        upstream_publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr upstream_publisher_;
    rclcpp::Publisher<general_interfaces::msg::ArmPose>::SharedPtr downstream_publisher_;

    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr upstream_subscription_;
    rclcpp::Subscription<general_interfaces::msg::ArmPose>::SharedPtr downstream_subscription_;

    std::vector<double> referencePositions_;
    std::vector<double> armPositions_;

    LinearActuator LA1_;
    LinearActuator LA2_;

    bool testing;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKHardwareInterface>());
  rclcpp::shutdown();
  return 0;
}
