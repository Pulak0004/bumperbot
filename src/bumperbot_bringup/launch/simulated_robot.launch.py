import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_description"), "launch", "gazebo.launch.py")
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_localization"), "launch", "local_localization.launch.py")
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_controller"), "launch", "controller.launch.py"),
        launch_arguments={"use_simple_controller": "False"}.items()
    )

    joystick = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_controller"), "launch", "joystick_controller.launch.py")
    )

    return LaunchDescription([
        gazebo,
        local_localization,
        controller,
        joystick
    ])






