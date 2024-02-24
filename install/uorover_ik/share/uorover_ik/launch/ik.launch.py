from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("crane", package_name="uorover_moveit_config").to_moveit_configs()
    launch_description =  generate_demo_launch(moveit_config)

    moveit_config = MoveItConfigsBuilder("uorover").to_moveit_configs()

    ik_launch = Node(
        name="uorover_ik",
        package="uorover_ik",
        executable="uorover_ik",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    hardware_interface = Node(
            name="ik_hardware_interface",
            package="uorover_ik",
            executable="ik_hardware_interface",
            output="screen"
            )

    launch_description.add_action(ik_launch)
    launch_description.add_action(hardware_interface)

    return launch_description
