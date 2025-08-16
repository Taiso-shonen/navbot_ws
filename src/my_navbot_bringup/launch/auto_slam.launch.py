import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    params_file_path = os.path.join(get_package_share_path('my_navbot_bringup'), 'config', 'auto_nav_params.yaml')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Bool value used with gazebo'
    )

    declare_map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Path to param file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')
    params_file = LaunchConfiguration('params_file')

    my_slam_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_path('my_navbot_bringup'), 'launch', 'slam.launch.py'),
        launch_arguments={'map_name': map_name}.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_path('nav2_bringup'), 'launch', 'navigation_launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file}.items()
    )

    nav2_mapping_node = Node(
        package="my_navbot_tools",
        executable="nav2_mapping"
    )

    nav_to_pose_client_node = Node(
        package="my_navbot_tools",
        executable="nav_to_pose_client",
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_map_name_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(my_slam_launch)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(nav2_mapping_node)
    ld.add_action(nav_to_pose_client_node)

    return ld
