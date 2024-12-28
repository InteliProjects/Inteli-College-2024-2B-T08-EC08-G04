#! /usr/bin/env python3 
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi
import time

rclpy.init()
nav = BasicNavigator()
q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
initial_pose = PoseStamped()

def create_pose_stamped(navigator, pos_x, pos_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = pos_x
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = nav.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.position.z = 0.0
initial_pose.pose.orientation.x = q_x
initial_pose.pose.orientation.y = q_y
initial_pose.pose.orientation.z = q_z
initial_pose.pose.orientation.w = q_w

nav.setInitialPose(initial_pose)
nav.waitUntilNav2Active()

goal_pose1 = create_pose_stamped(nav, 1.45, -0.5, 0)
goal_pose2 = create_pose_stamped(nav, 0.84, 0.35, 0)
goal_pose3 = create_pose_stamped(nav, 0.0, 0.0, 0.2)

waypoints = [goal_pose1, goal_pose2, goal_pose3]

nav.followWaypoints(waypoints)
while not nav.isTaskComplete():
    print(nav.getFeedback())


rclpy.shutdown()