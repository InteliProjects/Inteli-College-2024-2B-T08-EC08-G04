from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from typing import List

class RobotNavigator:
    def __init__(self, initial_x: float, initial_y: float, initial_z: float):
        self.navigator = BasicNavigator()
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
        
        # Set initial position
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = initial_x
        self.initial_pose.pose.position.y = initial_y
        self.initial_pose.pose.position.z = initial_z
        self.initial_pose.pose.orientation.x = q_x
        self.initial_pose.pose.orientation.y = q_y
        self.initial_pose.pose.orientation.z = q_z
        self.initial_pose.pose.orientation.w = q_w
        
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

        # Define waypoints
        self.positions = [
            self.create_pose_stamped(self.navigator, 1.45, -0.5, 0.0),
            self.create_pose_stamped(self.navigator, 0.84, 0.35, 0.0),
            self.create_pose_stamped(self.navigator, 0.0, 0.0, 0.2),
        ]

    def create_pose_stamped(self, navigator, pos_x: float, pos_y: float, rot_z: float) -> PoseStamped:
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pos_x
        pose.pose.position.y = pos_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def go_to_position(self, position_index: int):
        if position_index < 0 or position_index >= len(self.positions):
            raise ValueError("Invalid position index")
        goal_pose = self.positions[position_index]
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(feedback)
        result = self.navigator.getResult()
        return result
