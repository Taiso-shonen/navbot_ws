import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    rviz_config = os.path.join(get_package_share_path('my_navbot_description'), 'rviz', 'navbot_slam_config.rviz')
    world = os.path.join(get_package_share_path('my_navbot_bringup'), 'worlds', 'test_world.sdf')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Bool value used with gazebo'
    )

    declare_map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')

    my_navbot_gazebo_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_path('my_navbot_bringup'), 'launch', 'my_navbot_gazebo.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'world': world, 'rviz_config': rviz_config}.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
                os.path.join(get_package_share_path('slam_toolbox'), 'launch', 'online_async_launch.py'),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
    
    nav2_map_saver_launch = IncludeLaunchDescription(
                os.path.join(get_package_share_path('nav2_map_server'), 'launch', 'map_saver_server.launch.py'),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )

    my_map_saver_node = Node(
        package="my_navbot_tools",
        executable="my_map_saver",
        parameters=[{'map_name': map_name}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_map_name_arg)
    ld.add_action(my_navbot_gazebo_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_map_saver_launch)
    ld.add_action(my_map_saver_node)

    return ld
