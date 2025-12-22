from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import TimerEvent
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()
    robot_description_file = os.path.join(
        get_package_share_directory('dofbot_moveit_config'), 'config', 'dofbot.urdf.xacro'
    )
    robo_des_raw = xacro.process_file(robot_description_file).toxml()
    joint_controllers_file = os.path.join(
        get_package_share_directory('dofbot_moveit_config'), 'config', 'ros2_controllers.yaml'
    )

    world_path = PathJoinSubstitution([
        FindPackageShare("dofbot_description"),
        "worlds",
        "world.sdf"
    ])
    
    robot_description = Command(['xacro', robot_description_file])

    moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit_config")
        .robot_description(file_path="config/dofbot.urdf.xacro")
        .robot_description_semantic(file_path="config/dofbot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    x_args = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_args = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_args = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("dofbot_moveit_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, joint_controllers_file],
        output="screen",
        remappings=[
            ("~/robot_description", '/robot_description')
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 🔁 Keep only /clock from Gazebo, remove sim RGBD camera bridges
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # 🧭 Static TF: robot → camera_link_optical (used by your custom pointcloud)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '-0.05', '0.015', '0.01',              # x, y, z offset (tune as needed)
            '0.0', '0.0', '-1.57',      # roll, pitch, yaw in radians
            'DaBai_DCW2_Link',          # parent frame
            'camera_link_optical'       # child frame (used in pointcloud header)
        ]
    )

    publish_virtual_joint_once = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "/joint_states", "sensor_msgs/JointState",
            "{header: {stamp: {sec: 0, nanosec: 0}}, name: ['vj1_joint', 'vj2_joint'], position: [0.0, 0.0]}",
            "--once"
        ],
        output="screen"
    )

    use_sim_time = {"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[gripper_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[rviz_node],
        )
    )

    # Add this to your launch file
    joint_state_debug = Node(
        package='dofbot_moveit_config',
        executable='joint_state_debug',
        name='joint_state_debug',
        output='screen'
    )

    ld.add_action(x_args)
    ld.add_action(y_args)
    ld.add_action(z_args)
    ld.add_action(gazebo)
    ld.add_action(ros2_control_node)
    ld.add_action(gz_spawn_entity)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(move_group_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_rviz_node)
    ld.add_action(gz_sim_bridge)
    ld.add_action(publish_virtual_joint_once)
    ld.add_action(static_tf_node)
    # joint_state_debug,

    return ld
