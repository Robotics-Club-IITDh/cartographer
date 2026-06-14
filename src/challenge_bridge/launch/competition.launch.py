import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bridge_dir = get_package_share_directory('challenge_bridge')
    world_file = os.path.join(bridge_dir, 'worlds', 'training_world_2.sdf')
    robot_model_path = os.path.join(bridge_dir, 'models', 'robot.sdf')
    
    # 1. Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. Spawn Robot 1
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', robot_model_path, '-name', 'robot_1', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 3. Spawn Robot 2
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', robot_model_path, '-name', 'robot_2', '-x', '2.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 4. Global Shared Bridge (Clock)
    global_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 5. Dedicated Bridge for Robot 1
    robot_1_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_1_bridge',
        arguments=[
            '/model/robot_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/robot_1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/training_world_2/model/robot_1/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/robot_1/cmd_vel', '/robot_1/cmd_vel'),
            ('/model/robot_1/odometry', '/robot_1/odom'),
            ('/model/robot_1/tf', '/tf'),
            ('/world/training_world_2/model/robot_1/link/lidar_link/sensor/gpu_lidar/scan', '/robot_1/scan'),
        ],
        output='screen'
    )

    # 6. Dedicated Bridge for Robot 2
    robot_2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_2_bridge',
        arguments=[
            '/model/robot_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/robot_2/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/training_world_2/model/robot_2/link/lidar_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/robot_2/cmd_vel', '/robot_2/cmd_vel'),
            ('/model/robot_2/odometry', '/robot_2/odom'),
            ('/model/robot_2/tf', '/tf'),
            ('/world/training_world_2/model/robot_2/link/lidar_link/sensor/gpu_lidar/scan', '/robot_2/scan'),
        ],
        output='screen'
    )
    
    # Static Transforms
    robot_1_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_1_static_tf_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.2', '0.0', '0.1',      # x, y, z
            '0.0', '0.0', '0.0',       # yaw, pitch, roll (or use 0.0 if flat)
            'robot_1/base_link',       # Parent Frame
            'robot_1/lidar_link/gpu_lidar'       # Child Frame
        ]
    )

    robot_2_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_2_static_tf_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.2', '0.0', '0.1',      # x, y, z (Matches robot 1 specifications)
            '0.0', '0.0', '0.0',       # yaw, pitch, roll
            'robot_2/base_link',       # Parent Frame
            'robot_2/lidar_link/gpu_lidar'       # Child Frame
        ]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot1,
        spawn_robot2,
        global_bridge,
        robot_1_bridge,
        robot_2_bridge,
        robot_1_static_tf,
        robot_2_static_tf,
    ])