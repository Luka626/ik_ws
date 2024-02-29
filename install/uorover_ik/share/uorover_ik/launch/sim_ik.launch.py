from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rmm", package_name="uorover_moveit_config").to_moveit_configs()
    launch_description =  generate_demo_launch(moveit_config)

    ik_launch = Node(
        name="uorover_ik",
        package="uorover_ik",
        executable="uorover_ik",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )
    launch_description.add_action(ik_launch)

    hardware_interface = Node(
            name="ik_hardware_interface",
            package="uorover_ik",
            executable="ik_hardware_interface",
            output="screen"
    )
    #launch_description.add_action(hardware_interface)

    return launch_description
