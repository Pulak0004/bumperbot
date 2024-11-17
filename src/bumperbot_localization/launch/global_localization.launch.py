from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_path = PathJoinSubstitution([
        get_package_share_directory("bumperbot_mapping"), "maps", map_name, "map.yaml"
    ])
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},  # Example parameter
            {"yaml_filename": map_path}
        ]
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        nav2_map_server
    ])

