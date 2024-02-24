#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

int main(int argc, char * argv[])
{
  

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "uorover_ik",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("uorover_ik");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "uorover_arm");

  auto const target_pose = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.707099;
      msg.orientation.x = 0.706979;
      msg.orientation.y = 1e-6;
      msg.orientation.z = 1e-6;
      msg.position.x = 0.400;
      msg.position.y = -0.100;
      msg.position.z = 0.100;
      return msg;
  }();
  move_group_interface.setJointValueTarget(target_pose);
  move_group_interface.setGoalTolerance(1e-4);

  auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
  }();

  if(success){
      move_group_interface.execute(plan);
  } else{
      RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
