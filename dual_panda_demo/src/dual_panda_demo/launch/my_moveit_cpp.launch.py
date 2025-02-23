import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

    
def generate_launch_description():
    this_pkg_dir = get_package_share_directory("dual_panda_demo")
    moveit_config = (
        MoveItConfigsBuilder("dual_panda")
        .robot_description(file_path="config/dual_panda.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .moveit_cpp(file_path=os.path.join(this_pkg_dir, "config", "moveit_cpp.yaml"))
        .to_moveit_configs()
    )
    
    # Start the actual move_group node/action server
    moveit_cpp_demo = Node(
        package="dual_panda_demo",
        executable="my_moveit_cpp",
        output="log",
        parameters=[
            moveit_config.to_dict()
        ]
    )
    
    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("dual_panda_demo"),
        "config","moveit.rviz"
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
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )
    
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[moveit_config.robot_description, ros2_controllers_path],
    #     output="both",
    # )
    

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        # "L_manipulator_controller",
        # "R_manipulator_controller",
        "dual_manipulator_controller"
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]


    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            moveit_cpp_demo,
            ros2_control_node,
        ]
        + load_controllers
    )