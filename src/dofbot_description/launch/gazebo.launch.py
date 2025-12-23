import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # === Parameters/Arguments ===
    pkg_ur_description = 'dofbot_moveit_config'
    urdf_path = 'config/dofbot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_ur_description), urdf_path)
    robo_des_raw = xacro.process_file(xacro_file).toxml()

    world_path = PathJoinSubstitution([
        FindPackageShare("dofbot_description"),  # Replace with your package name
        "worlds",
        "world.sdf"
    ])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dofbot_description"), "config", "display.rviz"]
    )

    # === Nodes ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robo_des_raw},
            {'use_sim_time': True}
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_gz_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_gz_controller", "-c", "/controller_manager"],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robo_des_raw,
            "-name", "dofbot",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "-0.15",
            "-z", "1.1",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0"
        ]
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )


    depth_image_converter = Node(
        package="depth_image_proc",
        executable="convert_metric_node",
        name="depth_image_converter",
        output="screen",
        remappings=[
            ("image_raw", "/rgbd_camera/depth_image"),
            ("camera_info", "/rgbd_camera/camera_info"),
            ("image", "/rgbd_camera/depth_image/image")
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_sim_bridge,
        depth_image_converter,
    ])

