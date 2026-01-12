#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ==============================
    # PATHS (đường dẫn tuyệt đối)
    # ==============================
    map_yaml = '/home/tho/robot_ws/src/my_room_map.yaml'

    amcl_cfg       = '/home/tho/robot_ws/src/robot_bringup/config/amcl.yaml'
    planner_cfg    = '/home/tho/robot_ws/src/robot_bringup/config/planner.yaml'
    controller_cfg = '/home/tho/robot_ws/src/robot_bringup/config/controller.yaml'
    costmap_cfg    = '/home/tho/robot_ws/src/robot_bringup/config/costmap.yaml'
    btnav_cfg      = '/home/tho/robot_ws/src/robot_bringup/config/bt_navigator.yaml'
    behavior_cfg   = '/home/tho/robot_ws/src/robot_bringup/config/behavior.yaml'

    return LaunchDescription([

        # =====================================================
        # ROBOT CORE
        # =====================================================

        # Odometry
        Node(
            package='robot_odom',
            executable='odom_node',
            name='robot_odom',
            output='screen'
        ),

        # LiDAR pipeline (TF-Luna + stepper)
        Node(
            package='robot_scan',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),

        Node(
            package='robot_scan',
            executable='stepper_node',
            name='stepper_node',
            output='screen'
        ),

        Node(
            package='robot_scan',
            executable='scan_node',
            name='scan_node',
            output='screen'
        ),

        # =====================================================
        # TF
        # =====================================================

        # Static TF: base_link -> laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '0.0', '0.0', '0.15',
                '0.0', '0.0', '0.0',
                'base_link', 'laser_link'
            ]
        ),

        # =====================================================
        # NAV2 CORE
        # =====================================================

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': False
            }]
        ),

        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_cfg]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_cfg, costmap_cfg]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_cfg, costmap_cfg]
        ),

        # Behavior Server (RECOVERY – BẮT BUỘC)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[behavior_cfg]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[btnav_cfg]
        ),

        # =====================================================
        # LIFECYCLE MANAGER
        # =====================================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
