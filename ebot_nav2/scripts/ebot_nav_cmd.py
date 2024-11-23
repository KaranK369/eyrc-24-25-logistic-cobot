#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from geometry_msgs.msg import Quaternion


def quaternion_from_yaw(yaw):
    """Convert a yaw angle (in radians) to a quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


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

    # Define goal poses
    goal_poses = []

    # goal1
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.12
    goal_pose1.pose.position.y = -2.35
    goal_pose1.pose.orientation = quaternion_from_yaw(3.14)
    goal_poses.append(goal_pose1)

    # goal2
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.86
    goal_pose2.pose.position.y = 2.56
    goal_pose2.pose.orientation = quaternion_from_yaw(0.97)
    goal_poses.append(goal_pose2)

    # goal3
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.84
    goal_pose3.pose.position.y = 2.64
    goal_pose3.pose.orientation = quaternion_from_yaw(2.78)
    goal_poses.append(goal_pose3)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        # Implement some code here for your application!
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                print("Timeout reached. Cancelling navigation.")
                navigator.cancelTask()

    # Check the result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()
