import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory


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
            # left_moveit_wrapper,
            # right_moveit_wrapper,
        ]
        + load_controllers
    )