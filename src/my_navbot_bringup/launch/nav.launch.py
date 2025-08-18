import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    use_sim_time = 'true'
    rviz_config = os.path.join(get_package_share_path('my_navbot_description'), 'rviz', 'nav2_default_view.rviz')
    map_path = os.path.join(get_package_share_path('my_navbot_bringup'), 'maps', 'test_map.yaml')
    world_path = os.path.join(get_package_share_path('my_navbot_bringup'), 'worlds', 'test_world.sdf')
    waypoints_path = os.path.join(get_package_share_path('my_navbot_tools'), 'config', 'waypoints.yaml')
    params_file_path = os.path.join(get_package_share_path('my_navbot_bringup'), 'config', 'nav_params.yaml')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_path,
        description='Path to map'
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Path to world'
    )

    declare_initial_pose_arg = DeclareLaunchArgument(
        'set_initial_pose',
        default_value='true',
        description='Decides whether to call the initial pose setting function'
    )

    declare_waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value=waypoints_path,
        description='Path to waypoints yaml file'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Path to param file'
    )

    map = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    waypoints = LaunchConfiguration('waypoints')
    params_file = LaunchConfiguration('params_file')

    my_navbot_gazebo_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_path('my_navbot_bringup'), 'launch', 'my_navbot_gazebo.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'rviz_config': rviz_config}.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_path('nav2_bringup'), 'launch', 'bringup_launch.py'),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map, 'world': world, 'params_file': params_file}.items()
    )

    delayed_bringup = TimerAction(
        period=5.0,  # seconds
        actions=[nav2_bringup_launch]
    )

    nav_to_pose_client_node = Node(
        package="my_navbot_tools",
        executable="nav_to_pose_client",
        parameters=[{'set_initial_pose': set_initial_pose}]
    )

    waypoints_patrol_node = Node(
        package="my_navbot_tools",
        executable="waypoints_patrol",
        parameters=[{'yaml_path': waypoints}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_initial_pose_arg)
    ld.add_action(declare_waypoints_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(nav_to_pose_client_node)
    ld.add_action(waypoints_patrol_node)
    ld.add_action(my_navbot_gazebo_launch)
    ld.add_action(delayed_bringup)

    return ld
