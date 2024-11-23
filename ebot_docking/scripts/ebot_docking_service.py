#!/usr/bin/env python3

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        super().__init__('my_robot_docking_controller')
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Service
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize flags and parameters
        self.is_docking = False
        self.dock_aligned = False
        self.orientation_aligned = False  # Flag for orientation alignment
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.usrleft_value = float('inf')  # Left ultrasonic sensor value
        self.usrright_value = float('inf')  # Right ultrasonic sensor value
        self.target_distance = 0.0
        self.target_orientation = 0.0
        self.orientation_dock = False  # Flag for orientation docking
        self.linear_dock = False       # Flag for linear docking

        # Timer for control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Odometry callback to extract and update robot pose
    def odometry_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Ultrasonic sensor callback (left)
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Ultrasonic sensor callback (right)
    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    # Normalize angles between -π and π
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

 # Main control loop for docking
    def controller_loop(self):
        if self.is_docking:
            twist = Twist()

            if self.orientation_dock and not self.orientation_aligned:
                # Angular control (orientation docking)
                angular_error = self.target_orientation - self.robot_pose[2]
                angular_error = self.normalize_angle(angular_error)
                kp_angular = 2.0  # Proportional gain for angular control

                if abs(angular_error) > 0.01:  # Check if alignment is needed
                    twist.angular.z = kp_angular * angular_error
                    self.get_logger().info(f"Aligning orientation: error = {angular_error:.4f}")
                else:
                    twist.angular.z = 0.0
                    self.orientation_aligned = True  # Mark orientation as aligned
                    self.get_logger().info("Orientation aligned!")

            elif self.linear_dock and self.orientation_aligned:
                # Linear control (linear docking)
                distance_error = self.target_distance - statistics.mean([self.usrleft_value, self.usrright_value])
                kp_linear = 1.2  # Proportional gain for linear control

                if abs(distance_error) > 0.05:  # Check if alignment is needed
                    twist.linear.x = kp_linear * distance_error
                    self.get_logger().info(f"Aligning distance: error = {distance_error:.4f}")
                else:
                    twist.linear.x = 0.0
                    self.dock_aligned = True  # Mark docking as complete
                    self.get_logger().info("Docking complete!")

            # Publish the velocity command
            self.vel_pub.publish(twist)

            # Stop the docking process once everything is aligned
            if self.dock_aligned:
                self.is_docking = False
                self.get_logger().info("Docking alignment complete!")



    # Service callback for DockControl
    def dock_control_callback(self, request, response):
        self.target_distance = request.distance
        self.target_orientation = request.orientation
        self.orientation_dock = request.orientation_dock
        self.linear_dock = request.linear_dock

        # Reset flags
        self.is_docking = True
        self.dock_aligned = False
        self.orientation_aligned = not self.orientation_dock  # Skip if no orientation docking required

        # Log a message indicating docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until docking is complete
        while not self.dock_aligned:
            rate.sleep()

        # Respond with success message
        response.success = True
        response.message = "Docking complete"
        return response

# Main function to initialize the ROS2 node
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
