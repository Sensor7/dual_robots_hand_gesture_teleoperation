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
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import threading
import time
import tf2_ros
import tf2_geometry_msgs
from arm_api2_py.arm_api2_client import ArmAPI2Client
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from arm_api2_msgs.srv import PlanPath
from geometry_msgs.msg import TwistStamped
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Empty, Trigger

linear_step_size = np.zeros(3)
x_angle, y_angle, z_angle = 0.0, 0.0, 0.0
left_move_robot = False
right_move_robot = False

class FrankaServoNode(Node):
    def __init__(self):
        super().__init__('robot_servo_python')
        self.callback_group = ReentrantCallbackGroup()
        
        # Create services
        self.left_waypoint_service = self.create_service(
            PlanPath, 
            'left_robot_waypoints', 
            self.left_waypoint_callback, 
            callback_group=self.callback_group
        )
        self.right_waypoint_service = self.create_service(
            PlanPath,
            'right_robot_waypoints',
            self.right_waypoint_callback,
            callback_group=self.callback_group
        )
        
        # Get servo parameters
        self.get_servo_parameters()
        
        # Create trajectory publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.servo_params.command_out_topic,
            10
        )
        
        # Initialize servo and pose tracking
        self.setup_servo()
        
        # Initialize pose tracking with mutex
        self.pose_lock = threading.Lock()
        self.target_pose = self.servo.get_end_effector_pose()
        
        # Start pose tracking thread
        self.tracking_thread = threading.Thread(target=self.pose_tracker)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
        
        # Create command timer
        self.command_timer = self.create_timer(0.02, self.command_callback)  # 50Hz
        
        self.get_logger().info("Franka Servo node initialized")

    def get_servo_parameters(self):
        """Load servo parameters"""
        # In real implementation, you would load parameters from file or parameter server
        param_ns = "moveit_servo"
        # This is a simplified approach - in practice use parameters from YAML
        self.declare_parameter(f"{param_ns}.planning_frame", "panda_link0")
        self.declare_parameter(f"{param_ns}.command_out_topic", "/joint_trajectory")
        self.declare_parameter(f"{param_ns}.publish_period", 0.01)
        
        # In a real implementation, you would load all servo parameters 
        # from file using moveit_servo_params_from_yaml_file
        self.servo_params = ServoParameters()
        self.servo_params.planning_frame = self.get_parameter(f"{param_ns}.planning_frame").value
        self.servo_params.command_out_topic = self.get_parameter(f"{param_ns}.command_out_topic").value
        self.servo_params.publish_period = self.get_parameter(f"{param_ns}.publish_period").value

    def setup_servo(self):
        """Initialize the servo and pose tracking components"""
        # In a real implementation, you would:
        # 1. Create a planning scene monitor
        # 2. Initialize servo with parameters
        # 3. Setup pose tracking
        
        # For this conversion, we'll use a simplified representation
        self.servo = moveit_servo.Servo(
            self.node,  # Or equivalent in Python
            self.servo_params,
            self.planning_scene_monitor  # This would come from a proper initialization
        )
        
        # Set command type to POSE
        self.servo.set_command_type(moveit_servo.CommandType.POSE)

    def waypoint_callback(self, request, response):
        """Callback for the robot_waypoints service"""
        global linear_step_size, x_angle, y_angle, z_angle
        
        # Update linear step size
        linear_step_size = np.array([
            request.waypoint.pose.position.x,
            request.waypoint.pose.position.y,
            request.waypoint.pose.position.z
        ])
        
        # Update rotation angles
        x_angle = request.angles[0]
        y_angle = request.angles[1]
        z_angle = request.angles[2]
        
        return response

    def pose_tracker(self):
        """Thread function for pose tracking"""
        rate = self.create_rate(1.0 / self.servo_params.publish_period)
        
        while rclpy.ok():
            with self.pose_lock:
                # Get next joint state based on target pose
                joint_state = self.servo.get_next_joint_state(self.target_pose)
                
                # Check servo status
                status = self.servo.get_status()
                if status != moveit_servo.StatusCode.INVALID:
                    # Compose and publish trajectory message
                    msg = generate_joint_trajectory_message(
                        self.servo_params, 
                        joint_state
                    )
                    self.trajectory_pub.publish(msg)
            
            rate.sleep()

    def command_callback(self):
        """Timer callback to update the target pose"""
        global linear_step_size, x_angle, y_angle, z_angle
        
        with self.pose_lock:
            # Get current end effector pose
            current_pose = self.servo.get_end_effector_pose()
            
            # Apply translation
            # Note: This is a simplified version. In a real implementation,
            # you would use proper pose transformation objects
            new_pose = Pose()
            new_pose.position.x = current_pose.position.x + linear_step_size[0]
            new_pose.position.y = current_pose.position.y + linear_step_size[1]
            new_pose.position.z = current_pose.position.z + linear_step_size[2]
            
            # Apply rotation using quaternions
            # Note: This is simplified. In a real implementation, you would:
            # 1. Convert current orientation to quaternion
            # 2. Apply rotations in sequence (X, Y, Z)
            # 3. Set the resulting quaternion
            
            # For this conversion, we'll assume a helper function exists
            new_pose.orientation = self.apply_rotations(
                current_pose.orientation,
                x_angle, y_angle, z_angle
            )
            
            # Update target pose
            self.target_pose = new_pose

    def apply_rotations(self, current_orientation, x_angle, y_angle, z_angle):
        """
        Apply rotations to the current orientation
        This is a simplified representation - in a real implementation, 
        you would use quaternion operations via libraries like numpy-quaternion, 
        scipy.spatial.transform, or PyKDL
        """
        # In a real implementation, you would:
        # 1. Convert current_orientation to rotation matrix or quaternion
        # 2. Apply rotations in order (typically X, Y, Z)
        # 3. Convert back to quaternion
        
        # For now, we'll just return the current orientation as a placeholder
        return current_orientation

def main(args=None):
    # Sleep to match the C++ implementation
    time.sleep(0.75)
    
    rclpy.init(args=args)
    
    # Create and spin node
    franka_servo_node = FrankaServoNode()
    
    # Use MultiThreadedExecutor to handle callbacks from multiple threads
    executor = MultiThreadedExecutor()
    executor.add_node(franka_servo_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        franka_servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





















