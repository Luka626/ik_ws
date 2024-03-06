from launch_ros.actions import Node
import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import LaunchDescription
from moveit_configs_utils.moveit_configs_builder import get_package_share_directory

def generate_launch_description():

    moveit_config = (
            MoveItConfigsBuilder(
                "rmm",
                package_name="uorover_moveit_config"
                )
            .robot_description(mappings={
                "use_mock_hardware": "true",
                })
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True
            )
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner"]
            )
            .to_moveit_configs()
            )

    run_move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            )

    rviz_config_file = os.path.join(
            get_package_share_directory("uorover_moveit_config"),
            "config",
            "moveit.rviz",
            )
    
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                ],
            )
    static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["--frame-id", "base_footprint", "--child-frame-id", "L1"],
            )

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description],
            )
    ros2_controllers_path = os.path.join(
            get_package_share_directory("uorover_moveit_config"),
            "config",
            "ros2_controllers.yaml",
            )
    ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, ros2_controllers_path],
            )

    arm_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["uorover_arm_controller"],
            output="screen",
            )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
            )

    return LaunchDescription(
            [
                ros2_control_node,
                run_move_group_node,
                rviz_node,
                static_tf,
                robot_state_publisher,
                arm_controller_node,
                joint_state_broadcaster,
            ]
    )

