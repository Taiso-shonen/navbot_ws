import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('my_navbot_description'), 'urdf', 'my_navbot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('my_navbot_description'), 'rviz', 'navbot_config.rviz')
    world_path = os.path.join(get_package_share_path('my_navbot_bringup'), 'worlds', 'test_world.sdf')
    ros_gz_sim = get_package_share_path('ros_gz_sim')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Path to the RViz config file'
    )

    declare_rviz_flag = DeclareLaunchArgument(
    'launch_rviz',
    default_value='true',  # or 'false' if you prefer it off by default
    description='Whether to launch RViz'    
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Path to the World file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Bool value used with gazebo'
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')



    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    ros_gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    bridge_params = os.path.join(
        get_package_share_path('my_navbot_bringup'),
        'config',
        'gazebo_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen',
    )

    ros_gz_sim_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-name', 'navbot', '-x', '0', '-y', '0', '-z', '0', '-topic', 'robot_description'],
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(launch_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_flag)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros_gz_sim_launch)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(ros_gz_sim_node)
    ld.add_action(rviz2_node)

    return ld
