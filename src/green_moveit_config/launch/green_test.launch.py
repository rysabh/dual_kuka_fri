import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

launch_arguments = {
    "robot_ip": "192.168.10.116",
    "use_fake_hardware": "true",
    "dof": "7",
}

moveit_config = (
    MoveItConfigsBuilder(
        "green_iiwa7", package_name="green_moveit_config"
    )
    .robot_description(mappings=launch_arguments)
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    .to_moveit_configs()
)

run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output = "screen",
    parameters=[moveit_config.to_dict()],
)

rviz_config_arg = DeclareLaunchArgument(
    "rviz_config",
    default_value="green_config.rviz",
    description="Rviz configuration file",
)
rviz_base = LaunchConfiguration("rviz_config")
rviz_config = PathJoinSubstitution(
    [FindPackageShare("lbr_description"), "config", rviz_base]
)

rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config],
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
    ],
)

static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_state_publisher",
    output="log",
    arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
)

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],
)

ros2_controllers_path = os.path.join(
    get_package_share_directory("lbr_ros2_control"),
    "config",
    "lbr_controllers.yaml",
)
ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[ros2_controllers_path],
    remappings=[
        ("lbr/robot_description", "/robot_description"),
    ],
    output="both",
)

joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        "controller_manager",
    ],
)

arm_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
)

def generate_launch_description():

    # ... all our other code goes here

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            # ros2_controllers_path,
            run_move_group_node,
            # ros2_control_node,
            # joint_state_broadcaster_spawner,
            # arm_controller_spawner,
        ]
    )