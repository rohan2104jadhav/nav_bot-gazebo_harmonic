import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    # gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
    default_model_path = os.path.join(pkg_share, 'model', 'my_bot.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    # world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    # Start Gazebo (gz sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ' -r -v 4 empty.sdf'}.items()
    )


    # Bridge between Gazebo <-> ROS 2
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['@' + bridge_config_path],  # uses yaml config
        output='screen'
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
   )
   
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', default_model_path,
            '-name', 'my_bot',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )



    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                            description='Absolute path to rviz config file'),
                            
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        ros_gz_bridge,
        spawn_entity,
    ])
