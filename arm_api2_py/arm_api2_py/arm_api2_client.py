#!/usr/bin/env python3

import threading
import time
from typing import List
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController, ListControllers
from controller_manager_msgs.msg import ControllerState
from geometry_msgs.msg import TwistStamped
from arm_api2_msgs.action import MoveCartesian, MoveJoint, MoveCartesianPath
from arm_api2_msgs.srv import ChangeState, SetVelAcc, SetStringParam
from std_srvs.srv import SetBool


class ArmApi2Client:

    def __init__(self, node: Node, prefix: str = None):
        self._action_client_cartesian = ActionClient(
            node, MoveCartesian, f"{prefix}arm/move_to_pose")
        self._action_client_joint = ActionClient(
            node, MoveJoint, f"{prefix}arm/move_to_joint")
        self._action_client_cartesian_path = ActionClient(
            node, MoveCartesianPath, f"{prefix}arm/move_to_pose_path")
        self._action_client_gripper = ActionClient(
            node, GripperCommand, f"{prefix}arm/gripper_control")

        self._service_client_state_change = node.create_client(
            ChangeState, f"{prefix}arm/change_state")
        self._service_client_setvelacc = node.create_client(
            SetVelAcc, f"{prefix}arm/set_vel_acc")
        self._service_client_seteelink = node.create_client(
            SetStringParam, f"{prefix}arm/set_eelink")
        self._service_client_setplanonly = node.create_client(
            SetBool, f"{prefix}arm/set_planonly")
        self._service_client_switch_controller = node.create_client(
            SwitchController, "controller_manager/switch_controller")
        self._service_client_list_controllers = node.create_client(
            ListControllers, "controller_manager/list_controllers")

        self._publisher_twist_cmd = node.create_publisher(
            TwistStamped, "/moveit2_iface_node/delta_twist_cmds", 10)
        self._publisher_joint_vel_cmd = node.create_publisher(
            JointJog, "/moveit2_iface_node/delta_joint_cmds", 10)

        self._subscriber_current_ee_pose = node.create_subscription(
            PoseStamped, f"{prefix}arm/state/current_pose", self.current_pose_callback, 1)

        self.logger = node.get_logger()
        self._node = node
        self._current_ee_pose = PoseStamped()

    def current_pose_callback(self, msg):
        self._current_ee_pose = msg

    def get_current_ee_pose(self):
        """
        Returns the current end effector pose. EE link is set by the set_eelink service.

        Returns:
            PoseStamped: The current end effector pose.
        """
        return self._current_ee_pose

    def set_vel_acc(self, max_vel_scaling: float, max_acc_scaling: float):
        """
        Sends a request to the service server to set the velocity and acceleration scaling factors.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            max_vel_scaling (float): The maximum velocity scaling factor. In the range [0.0, 1.0].
            max_acc_scaling (float): The maximum acceleration scaling factor. In the range [0.0, 1.0].

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = SetVelAcc.Request()
        request.max_vel = max_vel_scaling
        request.max_acc = max_acc_scaling

        self.logger.info(" ArmApi2Client: Waiting for service server...")

        self._service_client_setvelacc.wait_for_service()

        self.logger.info(
            " ArmApi2Client: set_vel_acc request sent, waiting for response...")

        response = self._service_client_setvelacc.call(request)

        if response.success:
            self.logger.info(
                f" ArmApi2Client: Velocity and acceleration scaling \
                    factors set to {max_vel_scaling}, {max_acc_scaling}")
        else:
            self.logger.info(
                f" ArmApi2Client: Setting velocity and acceleration scaling \
                factors to {max_vel_scaling}, {max_acc_scaling} failed")

        return response.success

    def set_eelink(self, eelink_name: str):
        """
        Sends a request to the service server to set the end effector link name.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            eelink_name (str): The name of the end effector link.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = SetStringParam.Request()
        request.value = eelink_name

        self.logger.info(" ArmApi2Client: Waiting for service server...")

        self._service_client_seteelink.wait_for_service()

        self.logger.info(
            " ArmApi2Client: set_eelink request sent, waiting for response...")

        response = self._service_client_seteelink.call(request)

        if response.success:
            self.logger.info(
                f" ArmApi2Client: End effector link set to {eelink_name}")
        else:
            self.logger.info(
                f" ArmApi2Client: Setting end effector link to {eelink_name} failed")

        return response.success

    def set_planonly(self, plan_only: bool):
        """
        Sends a request to the service server to set the plan only flag.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.
        
        Args:
            plan_only (bool): The plan only flag.
            
        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = SetBool.Request()
        request.data = plan_only

        self.logger.info(" ArmApi2Client: Waiting for service server...")

        self._service_client_setplanonly.wait_for_service()

        self.logger.info(
            " ArmApi2Client: set_planonly request sent, waiting for response...")

        response = self._service_client_setplanonly.call(request)

        if response.success:
            self.logger.info(
                f" ArmApi2Client: Plan only flag set to {plan_only}")
        else:
            self.logger.info(
                f" ArmApi2Client: Setting plan only flag to {plan_only} failed")

        return response.success

    def change_state_to(self, state: str):
        """
        Sends a request to the service server to change the arm state to the specified state.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            state (str): The state to change the arm to. One of "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL"

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = ChangeState.Request()
        request.state = state

        self.logger.info(" ArmApi2Client: Waiting for service server...")

        self._service_client_state_change.wait_for_service()

        self.logger.info(
            " ArmApi2Client: change_state request sent, waiting for response...")

        response = self._service_client_state_change.call(request)

        if response.success:
            self.logger.info(f" ArmApi2Client: State changed to {state}")
        else:
            self.logger.info(f" ArmApi2Client: State change to {state} failed")

        return response.success

    def change_state_to_joint_ctl(self):
        """
        Sends a request to the service server to change the arm state to joint control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        res1 = self.switch_controller_for_joint_cartesian_ctl()
        res2 = self.change_state_to("JOINT_TRAJ_CTL")
        return res1 and res2

    def change_state_to_cartesian_ctl(self):
        """
        Sends a request to the service server to change the arm state to cartesian control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        # res1 = self.switch_controller_for_joint_cartesian_ctl()
        res2 = self.change_state_to("CART_TRAJ_CTL")
        return res2

    def change_state_to_servo_ctl(self):
        """
        Sends a request to the service server to change the arm state to servo control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """

        res1 = self.switch_controller_for_servo_ctl()
        res2 = self.change_state_to("SERVO_CTL")
        return res1 and res2
    
    def list_controllers(self):
        """
        Lists the available controllers in the UR ROS2 driver.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            List[str]: The list of available controllers.
        """
        request = ListControllers.Request()
        
        self.logger.info(" ArmApi2Client: Waiting for service server...")
        
        self._service_client_list_controllers.wait_for_service()
        
        self.logger.info(" ArmApi2Client: list_controllers request sent, waiting for response...")
        
        response = self._service_client_list_controllers.call(request)
        
        controllers = response.controller
        
        self.logger.info(f" ArmApi2Client: Available controllers: {len(controllers)}")
        
        return controllers
            

    def switch_controller(self, start_controllers: List[str], stop_controllers: List[str]):
        """
        Sends a request to the service server to switch controllers.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        reference about available controllers in UR ROS2 driver: 
        https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html#commanding-controllers

        Args:
            start_controllers (List[str]): The controllers to start.
            stop_controllers (List[str]): The controllers to stop.

        Returns:
            bool: True if the controllers were switched, False otherwise.
        """
        
        current_controllers = self.list_controllers()
        for controller in current_controllers:
            if controller.name  in start_controllers and controller.state == "active":
                self.logger.info(f" ArmApi2Client: Controller {controller.name} is already active")
                start_controllers.remove(controller.name)
            if controller.name in stop_controllers and controller.state == "inactive":
                self.logger.info(f" ArmApi2Client: Controller {controller.name} is already inactive")
                stop_controllers.remove(controller.name)
    
        
        if len(start_controllers) == 0 and len(stop_controllers) == 0:
            self.logger.info(" ArmApi2Client: No controllers to switch")
            return True
        
        request = SwitchController.Request()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        request.strictness = 2  # STRICT=2, BEST_EFFORT=1

        self.logger.info(" ArmApi2Client: Waiting for service server...")

        self._service_client_switch_controller.wait_for_service()

        self.logger.info(
            " ArmApi2Client: switch_controller request sent, waiting for response...")

        response = self._service_client_switch_controller.call(request)

        if response.ok:
            self.logger.info(f" ArmApi2Client: Controllers switched")
        else:
            self.logger.info(f" ArmApi2Client: Switching controllers failed")

        return response.ok

    def switch_controller_for_servo_ctl(self):
        """
        Sends a request to the service server to switch controllers to servo control.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the controllers were switched, False otherwise.
        """
        stop_controllers = ["scaled_joint_trajectory_controller"]
        start_controllers = ["forward_position_controller"]
        return self.switch_controller(start_controllers, stop_controllers)

    def switch_controller_for_joint_cartesian_ctl(self):
        """
        Sends a request to the service server to switch controllers to joint and cartesian control.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the controllers were switched, False otherwise.
        """
        stop_controllers = ["forward_position_controller"]
        start_controllers = ["scaled_joint_trajectory_controller"]
        return self.switch_controller(start_controllers, stop_controllers)

    def gripper_open(self):
        """
        Sends a request to the gripper action server to open the gripper.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        return self.send_gripper_command(0.0, 140.0)

    def gripper_close(self):
        """
        Sends a request to the gripper action server to close the gripper.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        return self.send_gripper_command(0.8, 140.0)

    def send_gripper_command(self, position: float, effort: float):
        """
        Sends a goal to the gripper action server to move the gripper to the specified position and effort.

        !!! IMPORTANT: This function is blocking until the goal is reached
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            position (float): The position to move the gripper to. In the range [0.0, 0.8].
            effort (float): The effort to apply to the gripper. In the range [20.0, 140.0].

        Returns:
            tuple: A tuple containing the following values: 
                - bool: True if the request was successful, False otherwise.
                - float: The position the gripper reached.
                - float: The effort the gripper reached.
                - bool: True if the gripper is stalled, False otherwise.
                - bool: True if the gripper reached the goal, False otherwise
        """

        command = GripperCommandMsg()
        command.position = position
        command.max_effort = effort

        goal_msg = GripperCommand.Goal()
        goal_msg.command = command

        self.logger.info(" ArmApi2Client: Waiting for action server...")

        self._action_client_gripper.wait_for_server()

        self.logger.info(
            " ArmApi2Client: send_gripper_command goal request sent, waiting for result...")

        result = self._action_client_gripper.send_goal(goal_msg)

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info(" ArmApi2Client: Goal reached")
            return True, result.result.position, result.result.effort, result.result.stalled, result.result.reached_goal
        else:
            self.logger.info(" ArmApi2Client: send_gripper_command failed")
            return False, 0.0, 0.0, False, False

    def send_joint_vel_cmd(self, joint_names: List[str], joint_velocities: List[float], base_frame: str = "base_link"):
        """
        Sends a joint velocity command to the joint velocity command topic.

        Args:
            joint_names (List[str]): The names of the joints to send the velocity command to.
            joint_velocities (List[float]): The velocities to send to the joints.
        """
        msg = JointJog()
        msg.joint_names = joint_names
        msg.velocities = joint_velocities
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        self._publisher_joint_vel_cmd.publish(msg)

    def send_twist_cmd(self, twist_stamped: TwistStamped):
        """
        Sends a twist command to the twist command topic.

        Args:
            TwistStamped_msg (TwistStamped): The twist command to send.
        """
        twist_stamped.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher_twist_cmd.publish(twist_stamped)

    def move_to_pose(self, goal_pose: PoseStamped):
        """
        Sends a goal to the action server to move the arm to the specified pose.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            goal_pose (PoseStamped): The goal pose to move the arm to.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        goal_msg = MoveCartesian.Goal()
        goal_msg.goal = goal_pose

        self.logger.info(" ArmApi2Client: Waiting for action server...")

        self._action_client_cartesian.wait_for_server()

        self.logger.info(
            " ArmApi2Client: move_to_pose goal request sent, waiting for result...")

        result = self._action_client_cartesian.send_goal(goal_msg)

        if result.result.success:
            self.logger.info(" ArmApi2Client: Goal reached")
        else:
            self.logger.info(" ArmApi2Client: move_to_pose failed")

        return result.result.success

    def move_to_joint(self, joint_positions: JointState):
        """
        Sends a goal to the action server to move the arm to the specified joint positions.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            joint_positions (JointState): The joint positions to move the arm to.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        goal_msg = MoveJoint.Goal()
        goal_msg.joint_state = joint_positions

        self.logger.info(" ArmApi2Client: Waiting for action server...")

        self._action_client_joint.wait_for_server()

        self.logger.info(
            " ArmApi2Client: move_to_joint goal request sent, waiting for result...")

        result = self._action_client_joint.send_goal(goal_msg)

        if result.result.success:
            self.logger.info(" ArmApi2Client: Goal reached")
        else:
            self.logger.info(" ArmApi2Client: move_to_joint failed")

        return result.result.success

    def move_to_pose_path(self, goal_path: List[PoseStamped]):
        """
        Sends a goal to the action server to move the arm to the specified poses in sequence.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            goal_poses (List[PoseStamped]): The goal poses to move the arm to.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        goal_msg = MoveCartesianPath.Goal()
        goal_msg.poses = goal_path

        self.logger.info(" ArmApi2Client: Waiting for action server...")

        self._action_client_cartesian_path.wait_for_server()

        self.logger.info(
            " ArmApi2Client: move_to_pose_path goal request sent, waiting for result...")

        result = self._action_client_cartesian_path.send_goal(goal_msg)

        if result.result.success:
            self.logger.info(" ArmApi2Client: Goal reached")
        else:
            self.logger.info(" ArmApi2Client: move_to_pose_path failed")

        return result
