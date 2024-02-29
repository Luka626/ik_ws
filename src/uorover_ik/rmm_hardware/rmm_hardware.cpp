#include "include/rmm_hardware/rmm_hardware.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <general_interfaces/msg/detail/arm_pose__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>


namespace rmm_hardware
{
    CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo & info){

        // if URDF cannot be read, return error
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
            return CallbackReturn::ERROR;

        }

        //initialize our 4 joints, and two state interfaces
        joint_position_.assign(4, 0);

        //.. and 1 command interface (position)
        joint_position_command_.assign(4, 0);

        // assign joint names
        for (const auto &joint : info_.joints){
            for (const auto & interface : joint.state_interfaces){
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }


        //initialize ros node
        node_ = rclcpp::Node::make_shared("rmm_hardware");

        arm_feedback_subscriber_ = node_->create_subscription<general_interfaces::msg::ArmPose>(
                "arm_feedback",
                rclcpp::SensorDataQoS(),
                std::bind(&ArmSystem::arm_callback, this, std::placeholders::_1));

        goal_state_publisher_ = node_->create_publisher<general_interfaces::msg::ArmPose>(
                "goal_states",
                rclcpp::QoS(1));

        LA1_.id        = 1;
        LA1_.lMax      = 6.00;
        LA1_.lMin      = 0.00;
        LA1_.thetaMax  = 77.00 * 0.0174533;
        LA1_.thetaMin  = -4.00 * 0.0174533;
        LA1_.slope     = 1.0 * (LA1_.lMax - LA1_.lMin) / (LA1_.thetaMax - LA1_.thetaMin);

        LA2_.id        = 2;
        LA2_.lMax      = 6.00;
        LA2_.lMin      = 0.00;
        LA2_.thetaMax  = 100.00 * 0.0174533;
        LA2_.thetaMin  = 0.00 * 0.0175433;
        LA2_.slope     = 1.0 * (LA2_.lMax - LA2_.lMin) / (LA2_.thetaMax - LA2_.thetaMin);

        return CallbackReturn::SUCCESS;
    };


    //for each joint, assign its state interface for pos/vel then export
    std::vector<hardware_interface::StateInterface> ArmSystem::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;

        int joint = TOWER;
        for (const auto &joint_name : joint_interfaces["position"]){
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[joint++]);
        }

        joint = TOWER;
        for (const auto &joint_name : joint_interfaces["velocity"]){
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[joint++]);
        }

        return state_interfaces;
    }
        
    //same for command interfaces (except we only command position right now)
    std::vector<hardware_interface::CommandInterface> ArmSystem::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int joint = TOWER;
        for (const auto &joint_name : joint_interfaces["position"]){
            command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[joint++]);
        }

        return command_interfaces;
    }


    // dummy control, return exactly what's commanded and write nothing
    hardware_interface::return_type ArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &){
        if (rclcpp::ok()){
            rclcpp::spin_some(node_);
        }
        return hardware_interface::return_type::OK;
    }

    void ArmSystem::arm_callback(const general_interfaces::msg::ArmPose &msg){
        for (std::size_t jnt = TOWER; jnt < LAST; jnt++){
            switch (jnt){
                case TOWER:
                    joint_position_[jnt] = msg.positions[jnt];
                    break;
                case SHOULDER:
                    joint_position_[jnt] = LA1_.thetaMin + (1/LA1_.slope) * (msg.positions[jnt] - LA1_.lMin);
                    break;
                case ELBOW:
                    joint_position_[jnt] = LA2_.thetaMax - (1/LA2_.slope) * (msg.positions[jnt] - LA2_.lMin);
                    break;
                case WRIST:
                    joint_position_[jnt] = msg.positions[jnt];
                    break;
            }
        };
    }

    hardware_interface::return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &){
        auto message = general_interfaces::msg::ArmPose();
        for (int jnt=TOWER; jnt<LAST; jnt++){
            switch (jnt){
                case TOWER:
                    message.positions.emplace_back(joint_position_command_[jnt]);
                    break;
                case SHOULDER:
                    message.positions.emplace_back(LA1_.lMin + LA1_.slope * (joint_position_command_[jnt] - LA1_.thetaMin));
                    break;
                case ELBOW:
                    message.positions.emplace_back(LA2_.lMax - LA2_.slope * (joint_position_command_[jnt] - LA2_.thetaMin));
                    break;
                case WRIST:
                    message.positions.emplace_back(joint_position_command_[jnt]);
                    break;
            }
        }

        if (rclcpp::ok()){
            goal_state_publisher_->publish(message);
        }

        return hardware_interface::return_type::OK;
    }
} // namespace uorover_ik


PLUGINLIB_EXPORT_CLASS(
       rmm_hardware::ArmSystem, hardware_interface::SystemInterface)
