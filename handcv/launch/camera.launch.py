from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (DeclareLaunchArgument, Shutdown, IncludeLaunchDescription,
                            SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration,
                                  Command, FindExecutable, PythonExpression)
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python import get_package_share_directory
import yaml
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            arguments=["-p framerate:=30.0 -p pixel_format:=rgb8"]
        ),
        Node(
            package="handcv",
            executable="handcv",
        ),
    ])
