import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    package_name = "moveit_resources_panda_description"
    rviz_name = "urdf.rviz"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    # urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    rviz_config_path = os.path.join(pkg_share, f"launch/{rviz_name}")

    robot_description = xacro.process_file(
        os.path.join(
            pkg_share,
            "urdf",
            "panda.urdf.xacro",
        )
    ).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # arguments=[urdf_model_path]
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        # arguments=[urdf_model_path]
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )
    
    ld.add_action(
        DeclareLaunchArgument(
            name="gui",
            default_value="true",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    # ld.add_action(DeclareLaunchArgument(name='model', default_value=urdf_model_path,
    #                                     description='Absolute path to robot urdf file')),

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)

    return ld