import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    # 1. Setup paths
    pkg_go2_config = get_package_share_directory('go2_config')
    # Use 'ign' command for Fortress on Ubuntu 22.04
    world_path = os.path.join(pkg_go2_config, 'worlds', 'default.sdf')
    single_robot_launch = os.path.join(pkg_go2_config, 'launch', 'gazebo_velodyne.launch.py')

    # 2. Start the Gazebo Simulator (Server + GUI) ONCE
    # We use 'ign gazebo' instead of 'gz sim' for Fortress compatibility
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    # 3. Define the 4 robots and their spawn coordinates
    robots = [
        {'name': 'go2_0', 'x': '0.0', 'y': '0.0'},
        {'name': 'go2_1', 'x': '1.5', 'y': '0.0'},
        {'name': 'go2_2', 'x': '0.0', 'y': '1.5'},
        {'name': 'go2_3', 'x': '1.5', 'y': '1.5'},
    ]

    spawn_cmds = []

    for robot in robots:
        spawn_cmds.append(
            GroupAction(
                actions=[
                    # Set the namespace for all nodes in this group
                    PushRosNamespace(robot['name']),

                    # A. Load Robot State Publisher and Champ Nodes
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(single_robot_launch),
                        launch_arguments={
                            'robot_name': robot['name'],
                            'world_init_x': robot['x'],
                            'world_init_y': robot['y'],
                            'world_init_z': '0.6',
                            'use_lidar': 'true',
                            'headless': 'true', # Prevents launching more Gazebo windows
                        }.items()
                    ),

                    # B. Spawn the Entity into the Gazebo World
                    # This pulls the URDF from the 'robot_description' topic
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        arguments=[
                            '-name', robot['name'],
                            '-topic', 'robot_description',
                            '-x', robot['x'],
                            '-y', robot['y'],
                            '-z', '0.6',
                        ],
                        output='screen',
                    ),

                    # C. Bridge for Lidar (Ignition -> ROS 2)
                    # Maps the internal Gazebo points to a namespaced ROS topic
                    Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                            f'/model/{robot["name"]}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
                        ],
                        remappings=[
                            (f'/model/{robot["name"]}/points', 'velodyne_points'),
                        ],
                        output='screen'
                    ),
                ]
            )
        )

    # Return the full launch description
    return LaunchDescription([
        start_gazebo,
        *spawn_cmds
    ])
