from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('multirobot_coordinator')
    
    # Config files
    scanner_config = os.path.join(pkg_dir, 'config', 'scanner_config.yaml')
    aruco_config = os.path.join(pkg_dir, 'config', 'aruco_config.yaml')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )
    
    # Nodes
    iiwa_scanner_node = Node(
        package='multirobot_coordinator',
        executable='iiwa_scanner_node',
        name='iiwa_scanner_node',
        output='screen',
        parameters=[
            scanner_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # ArUco detection node (multi-marker)
    aruco_node = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        output='screen',
        parameters=[
            aruco_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/image', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )
    
    # Task allocator node
    task_allocator_node = Node(
        package='multirobot_coordinator',
        executable='task_allocator_node',
        name='task_allocator_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Fra2mo executor node
    fra2mo_executor_node = Node(
        package='multirobot_coordinator',
        executable='fra2mo_executor_node',
        name='fra2mo_executor_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        iiwa_scanner_node,
        aruco_node,
        task_allocator_node,
        fra2mo_executor_node,
    ])
