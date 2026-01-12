from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # =========================
        # Odometry từ ESP32
        # =========================
        Node(
            package='robot_odom',
            executable='odom_node',
            name='robot_odom',
            output='screen'
        ),

        # =========================
        # TF-Luna LiDAR (Range)
        # =========================
        Node(
            package='robot_scan',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),

        # =========================
        # Stepper quay LiDAR 180°
        # =========================
        Node(
            package='robot_scan',
            executable='stepper_node',
            name='stepper_node',
            output='screen'
        ),

        # =========================
        # Gom Range + Angle → LaserScan
        # =========================
        Node(
            package='robot_scan',
            executable='scan_node',
            name='scan_node',
            output='screen'
        ),

        # =========================
        # Static TF: base_link → laser_link
        # =========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '0.0', '0.0', '0.15',   # x y z
                '0.0', '0.0', '0.0',    # roll pitch yaw
                'base_link',
                'laser_link'
            ]
        ),

        # =========================
        # SLAM Toolbox (TUNED cho LiDAR chậm)
        # =========================
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                # Frames
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',

                # Scan
                'scan_topic': '/scan',
                'throttle_scans': 1,

                # LiDAR
                'max_laser_range': 8.0,
                'minimum_laser_range': 0.1,

                # Timing (QUAN TRỌNG)
                'map_update_interval': 2.5,
                'minimum_time_interval': 2.2,
                'transform_timeout': 0.2,

                # SLAM
                'use_scan_matching': True,
                'use_sim_time': False,

            }]
        )
    ])
