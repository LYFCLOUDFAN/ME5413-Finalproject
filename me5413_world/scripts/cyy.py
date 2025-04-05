#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class MultiGoalNavigator:
    def __init__(self):
        rospy.init_node('multi_goal_navigator', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")

        # 目标点列表 [(x, y, yaw)]
        self.goals = [
                (21.04, -2.97, 0.0),
                (20.73, -9.08, 0.0),
                (22.28, -14.15, 0.0),
                (20.97, -21.00, 0.0)
        ]

    def send_goal(self, x, y, yaw):
        """ 发送目标点到 move_base """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # 仅考虑2D平面导航，yaw=0 时 w=1

        rospy.loginfo("Sending goal: ({}, {})".format(x, y))
        self.client.send_goal(goal)
        self.client.wait_for_result()  # 等待导航完成

        # 检查是否到达目标点
        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal ({}, {}) reached successfully!".format(x, y))
            return True
        else:
            rospy.logwarn("Failed to reach goal ({}, {}). State: {}".format(x, y, state))
            return False

    def start_navigation(self):
        """ 按顺序执行多个目标点 """
        for x, y, yaw in self.goals:
            success = self.send_goal(x, y, yaw)
            if not success:
                rospy.logwarn("Skipping to next goal due to failure.")

        rospy.loginfo("All goals processed.")

if __name__ == '__main__':
    navigator = MultiGoalNavigator()
    navigator.start_navigation()
