#!/usr/bin/env python3
import os
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    # --- 1. Define Paths ---
    go2_desc_pkg = get_package_share_directory("go2_description") 
    go2_config_pkg = get_package_share_directory("go2_config")
    
    world_file = os.path.join(go2_config_pkg, "worlds", "new.sdf") 
    # Using the standard robot.xacro where you added the custom LiDAR XML
    base_xacro_path = os.path.join(go2_desc_pkg, "xacro", "robot.xacro")
    
    # We will use this single, static file for ALL robots
    base_ros_control_path = os.path.join(go2_desc_pkg, "config", "ros_control", "ros_control.yaml")

    joints_config = os.path.join(go2_config_pkg, "config", "joints", "joints.yaml")
    gait_config = os.path.join(go2_config_pkg, "config", "gait", "gait.yaml")
    links_config = os.path.join(go2_config_pkg, "config", "links", "links.yaml")

    # --- 2. Inject Gazebo Environment Variables ---
    workspace_share_dir = os.path.join(get_package_prefix('go2_description'), 'share')

    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib'))
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=workspace_share_dir))
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=workspace_share_dir))

    # --- 3. Launch Gazebo Fortress ---
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 4 {world_file}"}.items(),
    )
    ld.add_action(gz_sim_cmd)

    # --- 3.5 Bridge Gazebo Clock to ROS 2 ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Humble ros_gz_bridge: topic@ros_type@gz_type (see ros_gz_bridge README)
        arguments=['/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    ld.add_action(clock_bridge)

    # --- 4. Multi-Robot Spawning ---
    ROWS = 2 
    COLS = 1 
    SPACING = 1.5  # Distance in meters between each robot

    for i in range(COLS):
        for j in range(ROWS):
            name = f"go2_{i}_{j}"
            namespace = f"/{name}"

            # Dynamically calculate this specific robot's X and Y spawn positions
            x_pos = float(i * SPACING)
            y_pos = float(j * SPACING)

            # Pre-compile the URDF, passing the STATIC YAML file directly
            compiled_urdf_path = os.path.join('/tmp', f"{name}.urdf")
            subprocess.run(
                [
                    'xacro', base_xacro_path, 
                    f'robot_name:={name}', 
                    f'ros_control_file:={base_ros_control_path}', 
                    '-o', compiled_urdf_path
                ], 
                check=True
            )

            # A. Launch the CHAMP controllers
            champ_bringup = GroupAction([
                PushRosNamespace(namespace=name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory("champ_bringup"), "launch", "bringup.launch.py")
                    ),
                    launch_arguments={
                        "description_path": compiled_urdf_path,
                        "joints_map_path": joints_config,
                        "links_map_path": links_config,
                        "gait_config_path": gait_config,
                        "use_sim_time": "True",
                        "robot_name": name, 
                        "gazebo": "true",
                        "lite": "false",
                        "rviz": "false",
                        "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                        "hardware_connected": "false",
                        "publish_foot_contacts": "false",
                        "close_loop_odom": "true",
                    }.items()
                )
            ])

            # B. Spawn the robot into Gazebo
            spawn_go2 = Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world",
                    "slam_world",
                    "-name",
                    name,
                    "-file",
                    compiled_urdf_path,
                    "-x",
                    str(x_pos),
                    "-y",
                    str(y_pos),
                    "-z",
                    "0.6",
                    "-Y",
                    "0.0",
                ],
                output="screen",
            )

            # C. Load Controllers
            spawn_broadcaster = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_states_broadcaster", "--controller-manager", f"{namespace}/controller_manager"],
                output="screen",
            )

            spawn_effort_controller = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_group_effort_controller", "--controller-manager", f"{namespace}/controller_manager"],
                output="screen",
            )

            # D. Bridge Gazebo PointCloud to ROS 2
            gz_lidar_topic = f'/world/slam_world/model/{name}/link/velodyne/sensor/velodyne-VLP16/scan/points'
            ros_lidar_topic = f'{namespace}/pointcloud'

            lidar_bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'{gz_lidar_topic}@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                ],
                parameters=[{'use_sim_time': True}],
                remappings=[
                    (gz_lidar_topic, ros_lidar_topic),
                ],
                output='screen',
            )

            # E. Link Robot Odom to Global Map (Fixes RViz Map Error)
            static_tf_map_to_odom = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "--x", str(x_pos), "--y", str(y_pos), "--z", "0.0",
                    "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                    "--frame-id", "map", "--child-frame-id", f"{name}/odom",
                ],
                output="screen",
            )

            # --- Execution Sequence ---
            delay_first_spawn = TimerAction(
                period=8.0,
                # Added the new bridge and tf2 nodes here
                actions=[champ_bringup, spawn_go2, lidar_bridge, static_tf_map_to_odom]
            )
            ld.add_action(delay_first_spawn)

            delay_controllers = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_go2,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[spawn_broadcaster, spawn_effort_controller]
                        )
                    ]
                )
            )
            ld.add_action(delay_controllers)

    return ld
