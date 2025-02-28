import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    package_name = "dual_panda_description"
    rviz_name = "view_robot.rviz"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    # urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    rviz_config_path = os.path.join(pkg_share, f"launch/{rviz_name}")

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )
    

    # ld.add_action(DeclareLaunchArgument(name='model', default_value=urdf_model_path,
    #                                     description='Absolute path to robot urdf file')),
    ld.add_action(rviz2_node)

    return ld