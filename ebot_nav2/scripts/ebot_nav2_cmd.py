#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from geometry_msgs.msg import Quaternion
from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw
import time


def quaternion_from_yaw(yaw):
    """Convert a yaw angle (in radians) to a quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def create_drop_point(navigator):
    """Create a drop point pose to be used in navigation."""
    drop_pose = PoseStamped()
    drop_pose.header.frame_id = 'map'
    drop_pose.header.stamp = navigator.get_clock().now().to_msg()
    drop_pose.pose.position.x = 0.43
    drop_pose.pose.position.y = -2.43
    drop_pose.pose.orientation = quaternion_from_yaw(3.14)
    return drop_pose

def receive_pkg(navigator):
    """Send a receive package request."""
    payload_req_cli = navigator.create_client(PayloadSW, '/payload_sw')
    while not payload_req_cli.wait_for_service(timeout_sec=1.0):
        navigator.get_logger().info('Payload service not available, waiting again...')
    
    req = PayloadSW.Request()
    req.receive = True
    req.drop = False
    payload_req_cli.call_async(req)

def drop_pkg(navigator):
    """Send a drop package request."""
    payload_req_cli = navigator.create_client(PayloadSW, '/payload_sw')
    while not payload_req_cli.wait_for_service(timeout_sec=1.0):
        navigator.get_logger().info('Payload service not available, waiting again...')
    
    req = PayloadSW.Request()
    req.receive = False
    req.drop = True
    payload_req_cli.call_async(req)


def docking(navigator):
    """Send a docking request and wait for the response."""
    docking_req_cli = navigator.create_client(DockSw, '/dock_control')
    while not docking_req_cli.wait_for_service(timeout_sec=1.0):
        navigator.get_logger().info('Docking service not available, waiting again...')
    
    req = DockSw.Request()
    req.linear_dock = True
    req.orientation_dock = True
    req.orientation = -1.57
    req.distance = 0.0
    
    # Call the service asynchronously and wait for the result
    future = docking_req_cli.call_async(req)
    rclpy.spin_until_future_complete(navigator, future)
    
    # Check if the docking service call succeeded
    if future.result() is not None:
        response = future.result()
        if response.success:
            navigator.get_logger().info(f'Docking succeeded: {response.message}')
        else:
            navigator.get_logger().error(f'Docking failed: {response.message}')
    else:
        navigator.get_logger().error('Failed to call docking service')

    # Add a final log to indicate the function is complete
    navigator.get_logger().info('Docking process completed.')


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.84
    initial_pose.pose.position.y = -9.05
    initial_pose.pose.orientation = quaternion_from_yaw(3.14)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Start navigation to the drop point first
    drop_point_pose = create_drop_point(navigator)
    navigator.goToPose(drop_point_pose)

    print("Navigating to drop point to receive package...")
    
    # Start timer
    start_time = time.time()

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print("Current status: Navigating to drop point")

    # Check if it reached the drop point
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached drop point, waiting 1 second before collecting package.')
        receive_pkg(navigator)
    else:
        print("Failed to reach drop point. Exiting.")
        navigator.lifecycleShutdown()
        exit(1)

    # Goal 1
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.32
    goal_pose1.pose.position.y = 2.55
    goal_pose1.pose.orientation = quaternion_from_yaw(-1.57)
    navigator.goToPose(goal_pose1)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print("Current status: Navigating to goal 1")

    # Dock after reaching goal 1
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached goal 1. Initiating docking...')
        docking(navigator)
        drop_pkg(navigator)
        time.sleep(1.3)
    else:
        print("Failed to reach goal 1. Exiting.")
        navigator.lifecycleShutdown()
        exit(1)

    # Return to drop point to collect another package
    drop_point_pose = create_drop_point(navigator)
    navigator.goToPose(drop_point_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print("Current status: Navigating to drop point")

    # Check if it reached the drop point again
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached drop point, waiting 1 second before collecting package.')
        receive_pkg(navigator)
    else:
        print("Failed to reach drop point. Exiting.")
        navigator.lifecycleShutdown()
        exit(1)

    # Goal 2
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -4.4
    goal_pose2.pose.position.y = 2.89
    goal_pose2.pose.orientation = quaternion_from_yaw(-1.57)
    navigator.goToPose(goal_pose2)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print("Current status: Navigating to goal 2")

    # Dock after reaching goal 2
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached goal 2. Initiating docking...')
        docking(navigator)
        drop_pkg(navigator)
    else:
        print("Failed to reach goal 2. Exiting.")
        navigator.lifecycleShutdown()
        exit(1)

    # End timer and calculate time taken
    end_time = time.time()
    time_taken = end_time - start_time
    print(f'Total time taken to complete the task: {time_taken:.2f} seconds')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
