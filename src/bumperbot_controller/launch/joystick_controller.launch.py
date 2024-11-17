from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():

    bumperbot_controller_pkg = get_package_share_directory("bumperbot_controller")

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(bumperbot_controller_pkg, "config", "joy_config.yaml")]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(bumperbot_controller_pkg, "config", "joy_teleop.yaml")]
    )


    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py"),
        launch_arguments={
            "cmd_vel_out": "bumperbot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_locks": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_joy": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_joy.yaml"),
        }.items()
    )

    return LaunchDescription([
        joy,
        joy_teleop,
        twist_mux_launch
    ])