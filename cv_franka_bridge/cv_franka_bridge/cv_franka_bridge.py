"""
Interpret gestures from the user, and convert waypoints into robot motion.

This node interprets four gestures from a human user: thumbs up, thumbs down,
closed fist, and open palm. These gestures are used to control whether or not
the robot tracks the position of the users hand and to control the gripper.
Waypoints received are transformed into the robot base link's frame. Two
PD loops, one for position and one for orientation, are used to control the robot.

SUBSCRIBERS:
  + /waypoint (PoseStamped) - The 3D location of the hand's pose.
  + /right_gesture (String) - The gesture that the right hand is making.
PUBLISHERS:
  + /text_marker (Marker) - The text marker that is published to the RViz.
SERVICE CLIENTS:
  + /robot_waypoints (PlanPath) - The service that plans and executes the robot's
    motion.
ACTION CLIENTS:
  + /panda_gripper/homing (Homing) - The action server that homes the gripper.
  + /panda_gripper/grasp (Grasp) - The action server that controls the gripper.

"""
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TwistStamped

from visualization_msgs.msg import Marker

from std_srvs.srv import Empty, Trigger
from std_msgs.msg import String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import JointState
from arm_api2_py.arm_api2_client import ArmApi2Client
from arm_api2_msgs.srv import PlanPath
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from scipy.spatial.transform import Rotation as R
import numpy as np

class CvFrankaBridge(Node):

    def __init__(self):
        super().__init__('cv_franka_bridge')

        # declare parameters
        self.declare_parameter('x_limits', "0.2,0.6")
        self.declare_parameter('y_limits', "-0.25,0.25")
        self.declare_parameter('z_limits', "0.2,0.7")

        # get parameters
        self.x_limits = [float(value) for value in self.get_parameter('x_limits').get_parameter_value().string_value.split(",")]
        self.y_limits = [float(value) for value in self.get_parameter('y_limits').get_parameter_value().string_value.split(",")]
        self.z_limits = [float(value) for value in self.get_parameter('z_limits').get_parameter_value().string_value.split(",")]

        # create callback groups
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()
        self.gesture_callback_group = MutuallyExclusiveCallbackGroup()

        # arm api2 wrapper for both arms
        self.left_arm_api2_client = ArmApi2Client(self, "left_")
        self.right_arm_api2_client = ArmApi2Client(self, "right_")

        # create subscribers
        self.left_waypoint_subscriber = self.create_subscription(PoseStamped, 'left_waypoint', self.left_waypoint_callback, 10, callback_group=self.waypoint_callback_group)
        self.right_waypoint_subscriber = self.create_subscription(PoseStamped, 'right_waypoint', self.right_waypoint_callback, 10, callback_group=self.waypoint_callback_group)
        self.left_gesture_subscriber = self.create_subscription(String, 'left_gesture', self.left_gesture_callback, 10, callback_group=self.gesture_callback_group)
        self.right_gesture_subscriber = self.create_subscription(String, 'right_gesture', self.right_gesture_callback, 10, callback_group=self.gesture_callback_group)

        # create publishers
        self.left_text_marker_publisher = self.create_publisher(Marker, 'left_text_marker', 10)
        self.right_text_marker_publisher = self.create_publisher(Marker, 'right_text_marker', 10)
        self.left_bounding_box_publisher = self.create_publisher(Marker, 'left_bounding_box', 10)
        self.right_bounding_box_publisher = self.create_publisher(Marker, 'right_bounding_box', 10)

        # create clients
        self.left_waypoint_client = self.create_client(PlanPath, 'left_robot_waypoints')
        self.left_waypoint_client.wait_for_service(timeout_sec=2.0)
        self.right_waypoint_client = self.create_client(PlanPath, 'right_robot_waypoints')
        self.right_waypoint_client.wait_for_service(timeout_sec=2.0)

        self.left_servo_client = self.create_client(Trigger, '/left_servo_node/start_servo')
        self.left_servo_client.wait_for_service(timeout_sec=2.0)
        self.right_servo_client = self.create_client(Trigger, '/right_servo_node/start_servo')
        self.right_servo_client.wait_for_service(timeout_sec=2.0)

        self.left_servo_shutdown_client = self.create_client(Trigger, '/left_servo_node/stop_servo')
        self.left_servo_shutdown_client.wait_for_service(timeout_sec=2.0)
        self.right_servo_shutdown_client = self.create_client(Trigger, '/right_servo_node/stop_servo')
        self.right_servo_shutdown_client.wait_for_service(timeout_sec=2.0)


        # create timer
        self.timer = self.create_timer(0.04, self.timer_callback)

        # # create action clients
        self.left_gripper_client = ActionClient(
                self, GripperCommand, 'left_hand_controller/gripper_cmd')
        self.right_gripper_client = ActionClient(
                self, GripperCommand, 'right_hand_controller/gripper_cmd')

        # create twist publisher
        self.left_twist_pub = self.create_publisher(
            TwistStamped,
            '/left_servo_node/delta_twist_cmds', 10)
        
        self.right_twist_pub = self.create_publisher(
            TwistStamped,
            '/right_servo_node/delta_twist_cmds', 10)

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create class variables
        self.left_text_marker = self.create_text_marker("Thumbs_up_to_begin_teleoperation", prefix="L_")
        self.right_text_marker = self.create_text_marker("Thumbs_up_to_begin_teleoperation", prefix="R_")
        self.left_gripper_ready = True
        self.right_gripper_ready = True
        self.left_gripper_status = "Open"
        self.right_gripper_status = "Open"
        self.left_gripper_force_control = False
        self.right_gripper_force_control = False
        self.left_gripper_force = 40.0
        self.right_gripper_force = 40.0
        self.max_gripper_force = 10.0

        self.left_current_waypoint = None
        self.left_previous_waypoint = None
        self.left_current_waypoint_quat = None
        self.left_offset = None
        self.left_initial_ee_pose = Pose(position=Point(x=0.30674, y=0.499969, z=0.590256),
                                        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
        self.left_desired_ee_pose = self.left_initial_ee_pose
        self.left_waypoints = []
        self.left_move_robot = False
        self.left_prev_gesture = None
        self.left_start_time = self.get_clock().now()

        self.right_current_waypoint = None
        self.right_previous_waypoint = None
        self.right_current_waypoint_quat = None
        self.right_offset = None
        self.right_initial_ee_pose = Pose(position=Point(x=0.30674, y=-0.5, z=0.59),
                                        orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
        self.right_desired_ee_pose = self.right_initial_ee_pose
        self.right_waypoints = []
        self.right_move_robot = False
        self.right_prev_gesture = None
        self.right_start_time = self.get_clock().now()

        self.left_desired_ee_pose_client = self.create_client(PlanPath, 'left_desired_ee_pose')
        self.left_desired_ee_pose_client.wait_for_service(timeout_sec=2.0)
        self.right_desired_ee_pose_client = self.create_client(PlanPath, 'right_desired_ee_pose')
        self.right_desired_ee_pose_client.wait_for_service(timeout_sec=2.0)


        self.start_time = self.get_clock().now()

        self.lower_distance_threshold = 3.0
        self.upper_distance_threshold = 10.0
        self.position_tolerance = [0.05, 0.05, 0.05, 10]
        self.angle_threshold = 5.0

        self.kp = 5.0
        self.ki = 0.0
        self.kd = 0.01
        self.kp_angle = 1.0
        self.ki_angle = 0.0
        self.kd_angle = 0.01
        self.max_output = 0.1
        self.integral_prior = 0
        self.position_error_prior = 0
        self.roll_error_prior = 0
        self.pitch_error_prior = 0
        self.yaw_error_prior = 0

        # bounding box variables
        self.x_limits = [0.2, 0.6]
        self.y_limits = [-0.25, 0.25]
        self.left_y_limits = [0.25, 0.75]
        self.right_y_limits = [-0.75, -0.25]
        self.z_limits = [0.2, 0.8]
        self.left_bounding_box_marker = self.create_box_marker(prefix="L_")
        self.right_bounding_box_marker = self.create_box_marker(prefix="R_")

        self.left_count = 0
        self.right_count = 0

        self.get_logger().info("CvFrankaBridge node initialized, follow the instructions to control the robot.")

    def activate_servo(self, prefix):
        """Activate the servo controller."""
        if prefix == "L_":
            future = self.left_servo_client.call_async(Trigger.Request())
            if future.done():
                self.get_logger().info("Left servo activated")
        elif prefix == "R_":
            future = self.right_servo_client.call_async(Trigger.Request())
            if future.done():
                self.get_logger().info("Right servo activated")
        else:
            future = self.left_servo_client.call_async(Trigger.Request())
            future = self.right_servo_client.call_async(Trigger.Request())

    def deactivate_servo(self, prefix):
        """Deactivate the servo controller."""
        if prefix == "L_":
            future = self.left_servo_shutdown_client.call_async(Trigger.Request())
            if future.done():
                self.get_logger().info("Left servo deactivated")
        elif prefix == "R_":
            future = self.right_servo_shutdown_client.call_async(Trigger.Request())
            if future.done():
                self.get_logger().info("Right servo deactivated")
        else:
            future = self.left_servo_shutdown_client.call_async(Trigger.Request())
            future = self.right_servo_shutdown_client.call_async(Trigger.Request())

    def create_text_marker(self, text, prefix=None):
        """Create a text marker."""
        marker = Marker()
        marker.header.frame_id = f"{prefix}panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text = text
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def create_box_marker(self, prefix=None):
        """Create a line strip that represents the bounding box."""
        marker = Marker()
        marker.header.frame_id = f"{prefix}panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.points = [
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[0])
                ]
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def send_gripper_commamd(self, prefix, position, effort):
        command = GripperCommandMsg()
        command.position = position
        command.max_effort = effort
        goal_msg = GripperCommand.Goal()
        goal_msg.command = command
        if prefix == "L_":
            future = self.left_gripper_client.send_goal_async(goal_msg)
            future.add_done_callback(lambda f: self.get_result_callback(f, prefix))
        elif prefix == "R_":
            future = self.right_gripper_client.send_goal_async(goal_msg)
            future.add_done_callback(lambda f: self.get_result_callback(f, prefix))
            

    def get_transform(self, target_frame, source_frame):
        """Get the transform between two frames."""
        try:
            trans = self.buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            return translation, rotation

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

    def get_ee_pose(self, prefix= None):
        """Get the current pose of the end-effector."""
        if prefix == "L_":
            return self.left_arm_api2_client.get_current_ee_pose().pose
        elif prefix == "R_":
            return self.right_arm_api2_client.get_current_ee_pose().pose
        else:
            return None

    def left_waypoint_callback(self, msg):
        """Callback for the waypoint subscriber."""
        if self.left_current_waypoint is None:
            self.left_current_waypoint = msg.pose
            self.left_previous_waypoint = msg.pose
            return

        distance = np.linalg.norm(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) -
                                  np.array([self.left_current_waypoint.position.x, self.left_current_waypoint.position.y, self.left_current_waypoint.position.z]))
        
        r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        r2 = R.from_quat([self.left_current_waypoint.orientation.x, self.left_current_waypoint.orientation.y, self.left_current_waypoint.orientation.z, self.left_current_waypoint.orientation.w])
        euler = r.as_euler('xyz', degrees=True)
        euler2 = r2.as_euler('xyz', degrees=True)

        angle_distance = np.linalg.norm(np.array(euler) - np.array(euler2))

        # filter out tiny movements to reduce jitter, and large errors from 
        # camera
        if distance < self.lower_distance_threshold and distance > self.upper_distance_threshold and angle_distance < self.angle_threshold:
            # experimental, might help with jerkiness when the use moves their hand too fast
            self.left_offset = self.left_current_waypoint
            return
        else:
            self.left_current_waypoint = msg.pose


    def right_waypoint_callback(self, msg):
        """Callback for the waypoint subscriber."""
        if self.right_current_waypoint is None:
            self.right_current_waypoint = msg.pose
            self.right_previous_waypoint = msg.pose
            return

        distance = np.linalg.norm(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) -
                                  np.array([self.right_current_waypoint.position.x, self.right_current_waypoint.position.y, self.right_current_waypoint.position.z]))

        r = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        r2 = R.from_quat([self.right_current_waypoint.orientation.x, self.right_current_waypoint.orientation.y, self.right_current_waypoint.orientation.z, self.right_current_waypoint.orientation.w])
        euler = r.as_euler('xyz', degrees=True)
        euler2 = r2.as_euler('xyz', degrees=True)

        angle_distance = np.linalg.norm(np.array(euler) - np.array(euler2))

        # filter out tiny movements to reduce jitter, and large errors from 
        # camera
        if distance < self.lower_distance_threshold and distance > self.upper_distance_threshold and angle_distance < self.angle_threshold:
            # experimental, might help with jerkiness when the use moves their hand too fast
            self.right_offset = self.right_current_waypoint
            return
        else:
            self.right_current_waypoint = msg.pose



    def left_gesture_callback(self, msg):
        """
        Callback for the left gesture subscriber.

        The left gesture is used to control the robot's motion and the gripper.
        
        Args:
        ----
        msg (String): The gesture that the left hand is making.

        Returns:
        -------
        None

        """
        if msg.data == "Thumb_Up" or msg.data == "Thumb_Down":
            # if thumbs up, start tracking the user's hand
            if self.left_count == 0:
                self.left_desired_ee_pose = self.get_ee_pose(prefix="L_")
                self.left_count += 1

            self.left_text_marker = self.create_text_marker(msg.data, prefix="L_")
            self.left_move_robot = False
            if msg.data == "Thumb_Up":
                self.activate_servo("L_")
            elif msg.data == "Thumb_Down":
                self.deactivate_servo("L_")

        elif msg.data == "Closed_Fist" and self.left_gripper_ready and self.left_gripper_status == "Open":
            # if closed fist, close the gripper
            self.left_text_marker = self.create_text_marker(msg.data, prefix="L_")
            self.left_gripper_ready = False
            self.left_gripper_force_control = False
            self.left_gripper_force = 40.0
            self.send_gripper_commamd("L_", 0.0, self.left_gripper_force)
            self.left_gripper_status = "Closed"

        elif msg.data == "Open_Palm" and self.left_gripper_ready and self.left_gripper_status == "Closed":
            # if open palm, open the gripper
            self.left_text_marker = self.create_text_marker(msg.data, prefix="L_")
            self.left_gripper_force = 3.0
            self.send_gripper_commamd("L_", 0.035, self.left_gripper_force)
            self.left_gripper_status = "Open"

        if msg.data != "Thumb_Up" and msg.data != "Thumb_Down":
            self.left_count = 0
        
        if self.left_prev_gesture == "Thumb_Up" and msg.data != "Thumb_Up":
            self.left_move_robot = True
            self.left_offset = self.left_current_waypoint
            self.left_initial_ee_pose = self.get_ee_pose(prefix="L_")
            phi = np.arctan2(self.left_desired_ee_pose.position.y, self.left_desired_ee_pose.position.x)
            quat = quaternion_from_euler(-np.pi, 0.0, 0.0)
            self.left_desired_ee_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        self.left_prev_gesture = msg.data
    
    def right_gesture_callback(self, msg):
        """
        Callback for the right gesture subscriber.

        The right gesture is used to control the robot's motion and the gripper.
        
        Args:
        ----
        msg (String): The gesture that the right hand is making.

        Returns:
        -------
        None

        """
        if msg.data == "Thumb_Up" or msg.data == "Thumb_Down":
            # if thumbs up, start tracking the user's hand
            if self.right_count == 0:
                self.right_desired_ee_pose = self.get_ee_pose(prefix="R_")
                self.right_count += 1

            self.right_text_marker = self.create_text_marker(msg.data, prefix="R_")
            self.right_move_robot = False
            if msg.data == "Thumb_Up":
                self.activate_servo("R_")
            elif msg.data == "Thumb_Down":
                self.deactivate_servo("R_")

        elif msg.data == "Closed_Fist" and self.right_gripper_ready and self.right_gripper_status == "Open":
            # if closed fist, close the gripper
            self.right_text_marker = self.create_text_marker(msg.data, prefix="R_")
            self.right_gripper_ready = False
            self.right_gripper_force_control = False
            self.right_gripper_force = 40.0
            self.send_gripper_commamd("R_", 0.0, self.right_gripper_force)
            self.right_gripper_status = "Closed"

        elif msg.data == "Open_Palm" and self.right_gripper_ready and self.right_gripper_status == "Closed":
            # if open palm, open the gripper
            self.right_text_marker = self.create_text_marker(msg.data, prefix="R_")
            self.right_gripper_force = 3.0
            self.send_gripper_commamd("R_", 0.035, self.right_gripper_force)
            self.right_gripper_status = "Open"

        if msg.data != "Thumb_Up" and msg.data != "Thumb_Down":
            self.right_count = 0
        
        if self.right_prev_gesture == "Thumb_Up" and msg.data != "Thumb_Up":
            self.right_move_robot = True
            self.right_offset = self.right_current_waypoint
            self.right_initial_ee_pose = self.get_ee_pose(prefix="R_")
            phi = np.arctan2(self.right_desired_ee_pose.position.y, self.right_desired_ee_pose.position.x)
            quat = quaternion_from_euler(-np.pi, 0.0, 0.0)
            self.right_desired_ee_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        self.right_prev_gesture = msg.data


    def get_result_callback(self, future, prefix):
        """Callback for the grasp result."""

        result = future.result()
        if prefix == "L_":
            self.left_gripper_ready = True
        elif prefix == "R_":
            self.right_gripper_ready = True
        else:
            self.left_gripper_ready = True
            self.right_gripper_ready = True

    def feedback_callback(self, feedback):
        """Callback for the feedback from the gripper action server."""

        self.get_logger().info(f"Feedback: {feedback}")

    async def timer_callback(self):
        """Callback for the timer."""
        # publish a text marker with the current gesture
        self.left_text_marker_publisher.publish(self.left_text_marker)
        self.left_bounding_box_publisher.publish(self.left_bounding_box_marker)
        self.right_text_marker_publisher.publish(self.right_text_marker)
        self.right_bounding_box_publisher.publish(self.right_bounding_box_marker)

        if self.left_move_robot:
            # find the end-effector's position relative to the offset, which was
            # set the last time the user made a thumbs up gesture
            delta = Pose()
            delta.position.x = (self.left_current_waypoint.position.x - self.left_offset.position.x) / 1000 # convert to meters
            delta.position.y = (self.left_current_waypoint.position.y - self.left_offset.position.y) / 1000 # convert to meters
            delta.position.z = (self.left_current_waypoint.position.z - self.left_offset.position.z) / 1000 # convert to meters

            # Get the current and desired positions and orientations of the end-effector
            self.left_desired_ee_pose.position.x = delta.position.z + self.left_initial_ee_pose.position.x
            self.left_desired_ee_pose.position.y = delta.position.x + self.left_initial_ee_pose.position.y
            self.left_desired_ee_pose.position.z = -delta.position.y + self.left_initial_ee_pose.position.z

        if (self.left_desired_ee_pose.position.x < self.x_limits[0] or self.left_desired_ee_pose.position.x > self.x_limits[1]):
            self.left_desired_ee_pose.position.x = self.x_limits[0] if self.left_desired_ee_pose.position.x < self.x_limits[0] else self.x_limits[1]
        if (self.left_desired_ee_pose.position.y < self.left_y_limits[0] or self.left_desired_ee_pose.position.y > self.left_y_limits[1]):
            self.left_desired_ee_pose.position.y = self.left_y_limits[0] if self.left_desired_ee_pose.position.y < self.left_y_limits[0] else self.left_y_limits[1]
        if (self.left_desired_ee_pose.position.z < self.z_limits[0] or self.left_desired_ee_pose.position.z > self.z_limits[1]):
            self.left_desired_ee_pose.position.z = self.z_limits[0] if self.left_desired_ee_pose.position.z < self.z_limits[0] else self.z_limits[1]

        if self.right_move_robot:
            # find the end-effector's position relative to the offset, which was
            # set the last time the user made a thumbs up gesture
            delta = Pose()
            delta.position.x = (self.right_current_waypoint.position.x - self.right_offset.position.x) / 1000
            delta.position.y = (self.right_current_waypoint.position.y - self.right_offset.position.y) / 1000
            delta.position.z = (self.right_current_waypoint.position.z - self.right_offset.position.z) / 1000

            # Get the current and desired positions and orientations of the end-effector
            self.right_desired_ee_pose.position.x = delta.position.z + self.right_initial_ee_pose.position.x
            self.right_desired_ee_pose.position.y = delta.position.x + self.right_initial_ee_pose.position.y
            self.right_desired_ee_pose.position.z = -delta.position.y + self.right_initial_ee_pose.position.z
        
        if (self.right_desired_ee_pose.position.x < self.x_limits[0] or self.right_desired_ee_pose.position.x > self.x_limits[1]):
            self.right_desired_ee_pose.position.x = self.x_limits[0] if self.right_desired_ee_pose.position.x < self.x_limits[0] else self.x_limits[1]
        if (self.right_desired_ee_pose.position.y < self.right_y_limits[0] or self.right_desired_ee_pose.position.y > self.right_y_limits[1]):
            self.right_desired_ee_pose.position.y = self.right_y_limits[0] if self.right_desired_ee_pose.position.y < self.right_y_limits[0] else self.right_y_limits[1]
        if (self.right_desired_ee_pose.position.z < self.z_limits[0] or self.right_desired_ee_pose.position.z > self.z_limits[1]):
            self.right_desired_ee_pose.position.z = self.z_limits[0] if self.right_desired_ee_pose.position.z < self.z_limits[0] else self.z_limits[1]

        try:
            left_ee_pose = self.get_ee_pose(prefix="L_")
            right_ee_pose = self.get_ee_pose(prefix="R_")
        except AttributeError as e:
            return

        self.PID_control(left_ee_pose, prefix="L_")
        self.PID_control(right_ee_pose, prefix="R_")
    

    def PID_control(self, ee_pose, prefix=None):
        if prefix == "L_":
            desired_ee_pose = self.left_desired_ee_pose
        elif prefix == "R_":
            desired_ee_pose = self.right_desired_ee_pose

        current_euler = list(euler_from_quaternion([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]))
        desired_euler = list(euler_from_quaternion([desired_ee_pose.orientation.x, desired_ee_pose.orientation.y, desired_ee_pose.orientation.z, desired_ee_pose.orientation.w]))

        # Orientation PID loops
        if current_euler[0] < 0:
            current_euler[0] += 2 * np.pi
        if desired_euler[0] < 0:
            desired_euler[0] += 2 * np.pi
        roll_error = desired_euler[0] - current_euler[0]
        pitch_error = desired_euler[1] - current_euler[1]
        yaw_error = desired_euler[2] - current_euler[2]

        roll_derivative = (roll_error - self.roll_error_prior)
        pitch_derivative = (pitch_error - self.pitch_error_prior)
        yaw_derivative = (yaw_error - self.yaw_error_prior)

        roll_output = self.kp_angle * roll_error - self.kd_angle * roll_derivative
        pitch_output = self.kp_angle * pitch_error + self.kd_angle * pitch_derivative
        yaw_output = self.kp_angle * yaw_error + self.kd_angle * yaw_derivative

        euler_output = [roll_output, -pitch_output, -yaw_output]

        self.roll_error_prior = roll_error
        self.pitch_error_prior = pitch_error
        self.yaw_error_prior = yaw_error

        # Position PID loop
        position_error = np.linalg.norm(np.array([desired_ee_pose.position.x, desired_ee_pose.position.y, desired_ee_pose.position.z]) -
                               np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]))

        derivative = (position_error - self.position_error_prior)
        output = self.kp * position_error + self.kd * derivative
        self.position_error_prior = position_error

        if output > self.max_output:
            output = self.max_output

        robot_move = PoseStamped()
        robot_move.header.frame_id = "world"
        robot_move.header.stamp = self.get_clock().now().to_msg()
        robot_move.pose.position.x = np.round(output * (desired_ee_pose.position.x - ee_pose.position.x),4)*10
        robot_move.pose.position.y = np.round(output * (desired_ee_pose.position.y - ee_pose.position.y),4)*10
        robot_move.pose.position.z = np.round(output * (desired_ee_pose.position.z - ee_pose.position.z),4)*10

        if prefix == "L_":

            planpath_request = PlanPath.Request()
            planpath_request.waypoint = robot_move
            planpath_request.angles = euler_output
            current_pose = self.left_arm_api2_client.get_current_ee_pose().pose
            if self.position_reached(current_pose, desired_ee_pose, self.position_tolerance):
                twist = TwistStamped()
                twist.header.frame_id = "world"
                twist.header.stamp = self.get_clock().now().to_msg()
                self.left_twist_pub.publish(twist)
            else:
                twist = TwistStamped()
                twist.header.frame_id = "world"
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.twist.linear.x = robot_move.pose.position.x
                twist.twist.linear.y = robot_move.pose.position.y
                twist.twist.linear.z = robot_move.pose.position.z
                if self.left_current_waypoint is not None:
                    quat = [self.left_current_waypoint.orientation.x, self.left_current_waypoint.orientation.y, self.left_current_waypoint.orientation.z, self.left_current_waypoint.orientation.w]
                    r = R.from_quat(quat)
                    euler = r.as_euler('xyz', degrees=True)
                    twist.twist.angular.x = - euler[0]/40
                    twist.twist.angular.y = euler[1]
                    twist.twist.angular.z = euler[2]
                self.left_twist_pub.publish(twist)

        elif prefix == "R_":
            planpath_request = PlanPath.Request()
            planpath_request.waypoint = robot_move
            planpath_request.angles = euler_output
            current_pose = self.right_arm_api2_client.get_current_ee_pose().pose
            # print("current_pose", current_pose)
            # print("desired_ee_pose", desired_ee_pose)
            if self.position_reached(current_pose, desired_ee_pose, self.position_tolerance):
                twist = TwistStamped()
                twist.header.frame_id = "world"
                twist.header.stamp = self.get_clock().now().to_msg()
                self.right_twist_pub.publish(twist)
            else:
                twist = TwistStamped()
                twist.header.frame_id = "world"
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.twist.linear.x = robot_move.pose.position.x
                twist.twist.linear.y = robot_move.pose.position.y
                twist.twist.linear.z = robot_move.pose.position.z
                if self.right_current_waypoint is not None:
                    quat = [self.right_current_waypoint.orientation.x, self.right_current_waypoint.orientation.y, self.right_current_waypoint.orientation.z, self.right_current_waypoint.orientation.w]
                    r = R.from_quat(quat)
                    euler = r.as_euler('xyz', degrees=True)
                    twist.twist.angular.x = - euler[0]/40
                    twist.twist.angular.y = euler[1]
                    twist.twist.angular.z = euler[2]
                self.right_twist_pub.publish(twist)

    def position_reached(self, current_pose, desired_pose, position_threshold=0.01):
        x_diff = abs(current_pose.position.x - desired_pose.position.x)
        y_diff = abs(current_pose.position.y - desired_pose.position.y)
        z_diff = abs(current_pose.position.z - desired_pose.position.z)
        current_quat = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        desired_quat = [desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w]

        current_rot = R.from_quat(current_quat).as_euler('xyz', degrees=True)[0]  # Extract roll (X-axis rotation)
        desired_rot = R.from_quat(desired_quat).as_euler('xyz', degrees=True)[0] 
        
        rot_diff = abs(current_rot - desired_rot)
        
        return (x_diff < position_threshold[0] and y_diff < position_threshold[1] and z_diff < position_threshold[2] and rot_diff < position_threshold[3])

def main(args=None):
    rclpy.init(args=args)

    cv_franka_bridge = CvFrankaBridge()

    rclpy.spin(cv_franka_bridge)


if __name__ == '__main__':
    main()





















