#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from std_msgs.msg import Bool


class MultiGroupNavigator:
    def __init__(self):
        rospy.init_node('multi_group_navigator', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")
        
        self.pub = rospy.Publisher('/cmd_open_bridge', Bool, queue_size=1)
        self.pub_cmd = rospy.Publisher('/percep/cmd', String, queue_size=10)
        self.red_sub = rospy.Subscriber("/percep/red", String, self.red_callback)
        self.min_sub = rospy.Subscriber("/percep/numbers", String, self.min_callback)
        self.target_sub = rospy.Subscriber("/percep/pose", PoseStamped, self.target_callback)

        self.red_bool = 0
        self.min_bool = 0
        self.cmd_msg = String()
        self.msg = Bool()
        self.msg.data = True

        self.x = 0.0
        self.y = 0.0
        self.x1 = 0.0
        self.y1 = 0.0

        # 目标点组 [(x, y, yaw)]
        self.goal_groups = [
            [
                (21.55, -1.78, -1.57),
                (20.73, -9.08, -1.57),
                (22.28, -14.15, -1.57),
                (20.97, -21.50, 3.14)
            ],
            [
                (10.18, -21.5, 1.57),
                (10.18, -3.07, 0.0),
                (19.0, -3.07, -1.57),
                (19.0, -21.5, 3.14)
            ],
            [
                (10.0, -8.00, 3.14),
                (10.0, -12.00, 3.14),
                (10.0, -16.00, 3.14),
            ],
            [
                (self.x + 1, self.y, 3.14),
            ],
            [
                (self.x - 1, self.y, 3.14),
                (self.x - 1, self.y, -1.57),
                (self.x - 1, self.y, 1.57),
            ],
            [
                (self.x - 1, self.y, 3.14),
                (self.x1 + 1, self.y1, 3.14),
            ]
        ]

        self.current_group_index = 0  # 当前目标点组索引
        self.current_goal_index = 0   # 当前组内目标点索引

    def red_callback(self, msg):
        if msg.data.strip().lower() == "red detected":  # 做个小清理 + 忽略大小写
            rospy.loginfo("Detected 'red'")
            self.red_bool = 1

    def min_callback(self, msg):
        if msg.data.strip().lower() == "1":  # 做个小清理 + 忽略大小写
            rospy.loginfo("Detected min")
            self.min_bool = 1


     def target_callback(self, msg):
        if self.red_bool == 1:  # 做个小清理 + 忽略大小写
            rospy.loginfo("Go 'red'")
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
        elif self.min_bool == 1:
            rospy.loginfo("Go 'min'")
            self.x1 = msg.pose.position.x
            self.y1 = msg.pose.position.y


    def send_goal(self, x, y, yaw, timeout=12):
        """ 发送目标点到 move_base，并定期检查状态 """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # 将 yaw 转换为四元数
        q = quaternion_from_euler(0, 0, yaw)  # 仅考虑2D平面导航，yaw 作为偏航角
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"Sending goal: ({x}, {y}, {yaw})")
        self.client.send_goal(goal)

        # 轮询状态，避免长时间卡住
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            state = self.client.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Goal ({x}, {y}, {yaw}) reached successfully!")
                return True
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                rospy.logwarn(f"Goal ({x}, {y}, {yaw}) failed with state: {state}")
                return False

            # 超时检测
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logwarn(f"Goal ({x}, {y}, {yaw}) timed out after {timeout} seconds!")
                self.client.cancel_goal()  # 取消当前目标
                return False

            rospy.sleep(1)  # 每秒检查一次状态

    def start_navigation(self):
        """ 执行多个目标点组 """
        while self.current_group_index < len(self.goal_groups) and not rospy.is_shutdown():
            current_group = self.goal_groups[self.current_group_index]
            
            if self.current_group_index == 1:
                self.cmd_msg = "count"
                seld.pub_cmd.publish(self.cmd_msg)
            if self.current_group_index == 2:
                self.cmd_msg = "red"
                seld.pub_cmd.publish(self.cmd_msg)
            if self.current_group_index == 3:
                self.msg.data = True  # 设置消息内容为 "open_bridge"
                rospy.loginfo("Publishing: %s", self.msg.data)
                self.pub.publish(self.msg)  # 发布消息
            if self.current_group_index == 4:
                self.cmd_msg = "min"
                seld.pub_cmd.publish(self.cmd_msg)


            while self.current_goal_index < len(current_group):
                x, y, yaw = current_group[self.current_goal_index]
                success = self.send_goal(x, y, yaw)

                if self.red_bool == 1:
                    self.red_bool = 0
                    break
                elif self.min_bool == 1:
                    self.min_bool = 0
                    break
                else:
                    self.current_goal_index += 1  # 继续下一个目标点

                if not success:
                    rospy.logwarn(f"Skipping to next goal due to failure at ({x}, {y})")

            rospy.loginfo(f"Finished Group {self.current_group_index + 1}!")

            # 切换到下一组
            self.current_group_index += 1
            self.current_goal_index = 0  # 目标索引重置，准备执行下一组

        rospy.loginfo("All goal groups completed! Navigation finished.")

if __name__ == '__main__':
    navigator = MultiGroupNavigator()
    navigator.start_navigation()
