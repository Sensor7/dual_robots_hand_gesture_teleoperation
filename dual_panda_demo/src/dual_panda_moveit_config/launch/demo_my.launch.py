import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("dual_panda")
        .robot_description(file_path="config/dual_panda.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("dual_panda_moveit_config"),
        "config/moveit.rviz",
    )

    print("moveit config:", moveit_config.robot_description_kinematics)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_panda_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="both",
    # )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        # parameters=[ros2_controllers_path],
        # remappings=[
        #     ("/controller_manager/robot_description", "/robot_description"),
        # ],
        output="both",
    )
    

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_arm_controller",
        "right_arm_controller",
        # "both_arm_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    left_servo_yaml = load_yaml("dual_panda_moveit_config", "config/left_panda_simulated_config.yaml")
    left_servo_params = {"moveit_servo": left_servo_yaml}
    
    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_servo_node",
        parameters= [
            left_servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            ],
        output="screen",
    )

    right_servo_yaml = load_yaml("dual_panda_moveit_config", "config/right_panda_simulated_config.yaml")
    right_servo_params = {"moveit_servo": right_servo_yaml}

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_servo_node",
        parameters= [
            right_servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            ],
        output="screen",
    )

    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #         {'use_intra_process_comms': True},
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/L_panda_link0", "frame_id": "/world"}],
            ),
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::JoyToServoPub",
            #     name="controller_to_servo_node",
            # ),
        ],
        output="screen",
    )
    # # arm_api2 moveit wrapper
    # left_moveit_wrapper = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare("arm_api2"), 
    #             'launch',
    #             'moveit2_iface.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'robot_name': 'franka_left',
    #     }.items()
    # )

    # right_moveit_wrapper = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare("arm_api2"), 
    #             'launch',
    #             'moveit2_iface.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'robot_name': 'franka_right',
    #     }.items()
    # )



    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            left_servo_node,
            right_servo_node,
            container,
        ]
        + load_controllers
    )