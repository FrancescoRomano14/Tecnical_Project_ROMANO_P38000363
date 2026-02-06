import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Set IGN_GAZEBO_SYSTEM_PLUGIN_PATH to include ign_ros2_control plugin
    ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )
    
    pkg_multirobot_control = get_package_share_directory('multirobot_control')
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_iiwa = get_package_share_directory('iiwa_description')
    
    # Aggiungi path modelli ArUco per Gazebo (usa multirobot_control)
    gz_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_multirobot_control, 'gazebo', 'models')
    )
    
    world_file_path = os.path.join(pkg_multirobot_control, 'worlds', 'empty_world_with_walls.world')
    xacro_file_fra2mo = os.path.join(pkg_multirobot_control, 'urdf', 'fra2mo_with_camera.urdf.xacro')  # Fra2mo solo LIDAR
    xacro_file_iiwa = os.path.join(pkg_multirobot_control, 'urdf', 'iiwa_with_camera.urdf.xacro')
    controllers_file_path = os.path.join(pkg_multirobot_control, 'config', 'iiwa_controllers.yaml')

    # Descrizione del robot fra2mo (solo LIDAR, senza camera)
    robot_description_fra2mo = ParameterValue(Command(['xacro ', xacro_file_fra2mo]), value_type=str)

    # Descrizione del robot iiwa con parametri ros2_control
    robot_description_iiwa = ParameterValue(
        Command([
            'xacro ', xacro_file_iiwa,
            ' use_sim:=true',
            ' use_fake_hardware:=false',
            ' runtime_config_package:=multirobot_control',
            ' controllers_file:=iiwa_controllers.yaml',
            ' namespace:=iiwa'
        ]),
        value_type=str
    )

    # Robot state publisher per fra2mo (configurazione originale con namespace)
    robot_state_publisher_fra2mo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_fra2mo',
        namespace='fra2mo',
        parameters=[{
            'robot_description': robot_description_fra2mo,
            'use_sim_time': True
        }]
    )

    # Robot state publisher per iiwa (nome DEVE essere "robot_state_publisher" per ign_ros2_control)
    robot_state_publisher_iiwa = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',  # Nome richiesto dal plugin
        namespace='iiwa',
        parameters=[{
            'robot_description': robot_description_iiwa,
            'use_sim_time': True
        }]
    )

    # Joint state publisher per fra2mo
    joint_state_publisher_fra2mo = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_fra2mo',
        namespace='fra2mo',
        parameters=[{'use_sim_time': True}]
    )

    # Joint state publisher per iiwa NON serve - usa controller_manager
    # joint_state_publisher_iiwa = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher_iiwa',
    #     namespace='iiwa',
    #     parameters=[{'use_sim_time': True}]
    # )

    # Avvio di Ignition Gazebo con il nostro mondo
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file_path, '-r'],
        output='screen'
    )

    # Spawn del fra2mo in Gazebo
    gz_spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_fra2mo',
        arguments=[
            '-topic', '/fra2mo/robot_description',
            '-name', 'fra2mo',
            '-x', '-2.5',
            '-y', '-2.5',
            '-z', '0.1',
            '-Y', '0.785'
        ],
        output='screen'
    )

    # Spawn dell'iiwa in Gazebo con posizione iniziale a L
    # Centro del mondo: base al centro, braccio a L con ultimo link orizzontale
    gz_spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_iiwa',
        arguments=[
            '-topic', '/iiwa/robot_description',
            '-name', 'iiwa',
            '-x', '-1.0',    # Spostato 1m verso ovest
            '-y', '0.0',     # Centro Y
            '-z', '0.3',     # Su piedistallo (0.3m altezza)
            '-j', 'joint_a1', '0.785',   # 45° (Nord-Est, evita marker Sud)
            '-j', 'joint_a2', '-0.5',    # Solleva spalla
            '-j', 'joint_a3', '0.0',     # Neutro
            '-j', 'joint_a4', '-1.57',   # -90° gomito (forma L)
            '-j', 'joint_a5', '0.0',     # Neutro  
            '-j', 'joint_a6', '0.0',     # Link_7 parallelo al piano
            '-j', 'joint_a7', '0.0'      # Neutro
        ],
        output='screen'
    )

    # Bridge per i topic ROS <-> Ignition
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Fra2mo odometry TF publisher (odom -> base_footprint)
    odom_tf_fra2mo = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace='fra2mo',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Controller spawners for iiwa
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,  # Attendi che Gazebo e controller_manager siano pronti
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/iiwa/controller_manager'],
                output='screen'
            )
        ]
    )

    iiwa_arm_controller_spawner = TimerAction(
        period=10.0,  # Dopo joint_state_broadcaster
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'iiwa_arm_controller',
                    '--controller-manager', '/iiwa/controller_manager',
                    '--param-file', controllers_file_path
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ign_plugin_path,  # Set environment variable first
        gz_model_path,  # ArUco models path
        robot_state_publisher_fra2mo,
        robot_state_publisher_iiwa,
        joint_state_publisher_fra2mo,
        gz_sim,
        gz_spawn_fra2mo,
        gz_spawn_iiwa,
        bridge,
        odom_tf_fra2mo,  # Fra2mo odometry TF
        joint_state_broadcaster_spawner,
        iiwa_arm_controller_spawner,
    ])
