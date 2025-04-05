#!/usr/bin/env python3

# 本程序支持三种工作模式：
# 1. "red" 模式：检测图像中的红色区域，若检测到红色则计算其中心并发布地图坐标。
# 2. "count" 模式：对区域内检测到的 box（数字）进行统计，将每个不同 box（中心距离相差大于0.1m）存入字典中。
# 3. "min" 模式：在当前视野中，对检测到的 box，根据在 "count" 模式下累计的 box 数量选择数量最少的那个，并发布其地图坐标。

import time
import cv2
import easyocr
import numpy as np
import rospy
import tf
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class Visual:
    def __init__(self, rate=10):
        rospy.loginfo(f"running rate: {rate}")
        # 默认模式设为 "count"，可通过话题切换为 "red" 或 "min"
        self.detect_mode = "red"
        self.bridge = CvBridge()
        self.rate = rate

        self.img_curr = None
        self.camera_info = rospy.wait_for_message("/front/camera_info", CameraInfo)
        self.intrinsic = np.array(self.camera_info.K).reshape(3, 3)
        self.projection = np.array(self.camera_info.P).reshape(3, 4)
        self.distortion = np.array(self.camera_info.D)
        self.img_frame = self.camera_info.header.frame_id
        self.ocr_detector = easyocr.Reader(["en"], gpu=True)
        self.curr_odom = None

        # 字典用于存储 "count" 模式下检测到的各数字的不同 box 信息
        # 结构示例： { 3: [ {"coordinate": (x1, y1), "count": N1},
        #                   {"coordinate": (x2, y2), "count": N2} ],
        #             7: [ {"coordinate": (x3, y3), "count": N3} ], ... }
        self.digit_box_dict = {}

        # ROS 发布器
        self.target_pose_pub = rospy.Publisher("/percep/pose", PoseStamped, queue_size=1)
        # 用于发布检测结果字符串，如 box 数量、数字字典或红色检测信息
        self.pubid = rospy.Publisher("/percep/numbers", String, queue_size=1)
        # 红色检测发布器
        self.red_pub = rospy.Publisher("/percep/red", String, queue_size=1)

        # ROS 订阅器
        self.tf_sub = tf.TransformListener()
        self.scansub = rospy.Subscriber("/front/scan", LaserScan, self.scan_callback)
        # 通过 "/rviz_panel/goal_name" 可切换模式，例如发送 "/mode red"、"/mode count" 或 "/mode min"
        self.goalsub = rospy.Subscriber("/rviz_panel/goal_name", String, self.goal_callback)
        # 通过 "/percep/cmd" 也可切换检测模式
        self.redcmdsub = rospy.Subscriber("/percep/cmd", String, self.detect_mode_callback)
        self.img_sub = rospy.Subscriber("/front/image_raw", Image, self.img_callback)
        self.odom_sub = rospy.Subscriber("/final_slam/odom", Odometry, self.odom_callback)
        rospy.loginfo("visual node initialized")

    def odom_callback(self, msg):
        self.curr_odom = msg

    def img_callback(self, msg: Image):
        self.img_curr = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detect_mode_callback(self, msg):
        # 外部消息内容应为 "red"、"count" 或 "min"
        self.detect_mode = msg.data
        rospy.loginfo(f"detect_mode switched to: {self.detect_mode}")

    def map_callback(self, msg):
        data = list(msg.data)
        for y in range(msg.info.height):
            for x in range(msg.info.width):
                i = x + (msg.info.height - 1 - y) * msg.info.width
                if data[i] >= 75:
                    data[i] = 100
                elif (data[i] >= 0) and (data[i] < 50):  # free
                    data[i] = 0
                else:  # unknown
                    data[i] = -1
        self.map = np.array(data).reshape(msg.info.height, msg.info.width)

    def goal_callback(self, msg):
        rospy.loginfo(f"Received goal command: {msg.data}")
        # 示例：命令格式 "/mode red" 或 "/mode count" 或 "/mode min"
        if msg.data[:5] == "/mode":
            self.detect_mode = msg.data.split()[1]
            rospy.loginfo(f"Switched detect mode to: {self.detect_mode}")

    def scan_callback(self, msg: LaserScan):
        self.scan_curr = msg.ranges
        self.scan_params_curr = [msg.angle_min, msg.angle_max, msg.angle_increment]

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.img_curr is None:
                continue
            rospy.loginfo_throttle(2, f"detect_mode: {self.detect_mode}")

            # 红色检测逻辑
            if self.detect_mode == "red":
                # 将 BGR 图像转换为 HSV
                hsv_image = cv2.cvtColor(self.img_curr, cv2.COLOR_BGR2HSV)

                lower_orange = np.array([10, 100, 20])
                upper_orange = np.array([25, 255, 255])
                mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

                # 调试显示 mask
                cv2.imshow("orange mask", mask)
                cv2.waitKey(1)

                if np.any(mask != 0):
                    self.red_pub.publish("red detected")
                    # 利用轮廓找到红色区域的中心
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            # 在图像上标记中心
                            cv2.circle(self.img_curr, (cx, cy), 5, (255, 0, 0), -1)
                            cv2.imshow("img", self.img_curr)
                            cv2.waitKey(1)
                            # 利用相机内参将中心像素转换到归一化相机坐标系
                            direction = np.array([[cx], [cy], [1]])
                            direction = np.dot(np.linalg.inv(self.intrinsic), direction)
                            p_in_cam = PoseStamped()
                            p_in_cam.header.frame_id = self.img_frame
                            p_in_cam.pose.position.x = direction[0].item()
                            p_in_cam.pose.position.y = direction[1].item()
                            p_in_cam.pose.position.z = direction[2].item()
                            p_in_cam.pose.orientation.w = 1
                            # 将该点从相机坐标系转换到激光雷达坐标系 "tim551"
                            self.tf_sub.waitForTransform("tim551", self.img_frame, rospy.Time.now(), rospy.Duration(1))
                            transformed = self.tf_sub.transformPose("tim551", p_in_cam)
                            yaw = np.arctan2(transformed.pose.position.y, transformed.pose.position.x)
                            # 利用激光雷达数据确定距离
                            idx = round((yaw - self.scan_params_curr[0]) / self.scan_params_curr[2])
                            distance = self.scan_curr[idx] - 0.6  # 补偿一定距离
                            angle = self.scan_params_curr[0] + idx * self.scan_params_curr[2]
                            x_lidar = distance * np.cos(angle)
                            y_lidar = distance * np.sin(angle)
                            p_in_lidar = PoseStamped()
                            p_in_lidar.header.frame_id = "tim551"
                            p_in_lidar.header.stamp = rospy.Time.now()
                            p_in_lidar.pose.position.x = x_lidar
                            p_in_lidar.pose.position.y = y_lidar
                            p_in_lidar.pose.position.z = 0
                            p_in_lidar.pose.orientation.w = 1
                            self.tf_sub.waitForTransform("/map", "tim551", rospy.Time.now(), rospy.Duration(1))
                            p_in_map = self.tf_sub.transformPose("/map", p_in_lidar)
                            print('coordinatre', p_in_map.pose.position.x, p_in_map.pose.position.y)
                            # 发布该红色区域对应的地图坐标作为目标位姿
                            goal_p = PoseStamped()
                            goal_p.header.frame_id = "map"
                            goal_p.header.stamp = rospy.Time.now()
                            goal_p.pose.position.x = p_in_map.pose.position.x
                            goal_p.pose.position.y = p_in_map.pose.position.y
                            goal_p.pose.position.z = 0
                            if self.curr_odom is not None:
                                goal_p.pose.orientation = self.curr_odom.pose.pose.orientation
                            else:
                                goal_p.pose.orientation.w = 1
                            self.target_pose_pub.publish(goal_p)
                else:
                    self.red_pub.publish("no orange")

            # 数字检测逻辑：支持 "count" 与 "min" 模式
            if self.detect_mode in ["count", "min"]:
                result = self.ocr_detector.readtext(self.img_curr, batch_size=2, allowlist="0123456789")
                img_show = self.img_curr.copy()
                box_count = 0
                # min 模式下候选列表，每个元素为 (digit, count, (x_map, y_map))
                candidate_list = []

                for detection in result:
                    # detection[0]: 文本检测框四个顶点
                    # detection[1]: 识别结果（单个数字）
                    # detection[2]: 置信度
                    if len(detection[1]) > 1:
                        continue
                    if detection[2] < 0.99:
                        continue

                    diag_vec = np.array(detection[0][2]) - np.array(detection[0][0])
                    diag_len = np.linalg.norm(diag_vec)
                    if diag_len < 60:
                        continue

                    # 绘制检测框及识别结果（调试用）
                    cv2.rectangle(
                        img_show,
                        (int(detection[0][0][0]), int(detection[0][0][1])),
                        (int(detection[0][2][0]), int(detection[0][2][1])),
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        img_show,
                        detection[1] + f" {detection[2]:.2f}",
                        (int(detection[0][0][0]), int(detection[0][0][1])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        img_show,
                        f"diag: {diag_len:.2f}",
                        (int(detection[0][0][0]), int(detection[0][0][1] - 40)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    
                    # 计算检测框中心像素
                    center = [(a + b) / 2 for a, b in zip(detection[0][0], detection[0][2])]
                    direction = np.array([[center[0]], [center[1]], [1]])
                    direction = np.dot(np.linalg.inv(self.intrinsic), direction)

                    p_in_cam = PoseStamped()
                    p_in_cam.header.frame_id = self.img_frame
                    p_in_cam.pose.position.x = direction[0].item()
                    p_in_cam.pose.position.y = direction[1].item()
                    p_in_cam.pose.position.z = direction[2].item()
                    p_in_cam.pose.orientation.w = 1

                    self.tf_sub.waitForTransform("tim551", self.img_frame, rospy.Time.now(), rospy.Duration(1))
                    transformed = self.tf_sub.transformPose("tim551", p_in_cam)
                    yaw = np.arctan2(transformed.pose.position.y, transformed.pose.position.x)

                    idx = round((yaw - self.scan_params_curr[0]) / self.scan_params_curr[2])
                    distance = self.scan_curr[idx] - 0.6
                    angle = self.scan_params_curr[0] + idx * self.scan_params_curr[2]
                    x_coord = distance * np.cos(angle)
                    y_coord = distance * np.sin(angle)

                    p_in_lidar = PoseStamped()
                    p_in_lidar.header.frame_id = "tim551"
                    p_in_lidar.header.stamp = rospy.Time.now()
                    p_in_lidar.pose.position.x = x_coord
                    p_in_lidar.pose.position.y = y_coord
                    p_in_lidar.pose.position.z = 0
                    p_in_lidar.pose.orientation.w = 1

                    self.tf_sub.waitForTransform("/map", "tim551", rospy.Time.now(), rospy.Duration(1))
                    p_in_map = self.tf_sub.transformPose("/map", p_in_lidar)
                    x_map = p_in_map.pose.position.x
                    y_map = p_in_map.pose.position.y

                    # 判断点的有效性和是否在预设区域内（区域可根据实际需要调整）
                    if x_map == np.inf or x_map == -np.inf or np.isnan(x_map) or \
                       y_map == np.inf or y_map == -np.inf or np.isnan(y_map):
                        continue
                    
                    # TODO: Change to our range
                    # if not (7 <= x_map <= 17 and -7.2 <= y_map <= 2):
                    #     continue

                    box_count += 1
                    try:
                        digit = int(detection[1])
                    except Exception as e:
                        rospy.logwarn(f"无法转换识别结果为整数: {detection[1]}，错误：{e}")
                        continue

                    # "count" 模式下：更新字典（同一数字的 box 若中心距离小于等于 0.1m 则认为是同一 box）
                    if self.detect_mode == "count":
                        if digit not in self.digit_box_dict:
                            self.digit_box_dict[digit] = []
                        found = False
                        for box in self.digit_box_dict[digit]:
                            bx, by = box["coordinate"]
                            dist_box = np.sqrt((x_map - bx)**2 + (y_map - by)**2)
                            if dist_box <= 0.1:
                                box["count"] += 1
                                # 更新为最新的坐标
                                box["coordinate"] = (x_map, y_map)
                                found = True
                                break
                        if not found:
                            self.digit_box_dict[digit].append({"coordinate": (x_map, y_map), "count": 1})
                    
                    # "min" 模式下：将当前视野中检测到的结果作为候选，
                    # 并记录 count 模式下该数字存储的 box 数量（若未存储，则设为一个较大值）
                    if self.detect_mode == "min":
                        if digit in self.digit_box_dict:
                            num_boxes = len(self.digit_box_dict[digit])
                        else:
                            num_boxes = 999
                        candidate_list.append((digit, num_boxes, (x_map, y_map)))
                    
                    cv2.imshow("img", img_show)
                    cv2.waitKey(1)

                if self.detect_mode == "count":
                    total_stored = sum(len(box_list) for box_list in self.digit_box_dict.values())
                    result_str = f"box count: {box_count}, Stored boxes: {total_stored}, Digit Dict: {self.digit_box_dict}"
                    self.pubid.publish(String(data=result_str))
                    rospy.loginfo(result_str)
                    print("Box types:", list(self.digit_box_dict.keys()))
                elif self.detect_mode == "min":
                    if candidate_list:
                        candidate = min(candidate_list, key=lambda x: x[1])
                        digit, count_boxes, coord = candidate
                        result_str = f"min digit: {digit}, Stored boxes: {count_boxes}, Coord: {coord}"
                        goal_p = PoseStamped()
                        goal_p.header.frame_id = "map"
                        goal_p.header.stamp = rospy.Time.now()
                        goal_p.pose.position.x = coord[0]
                        goal_p.pose.position.y = coord[1]
                        goal_p.pose.position.z = 0
                        if self.curr_odom is not None:
                            goal_p.pose.orientation = self.curr_odom.pose.pose.orientation
                        else:
                            goal_p.pose.orientation.w = 1
                        self.target_pose_pub.publish(goal_p)
                    else:
                        result_str = "min digit: None"
                    self.pubid.publish(String(data=result_str))
                    rospy.loginfo(result_str)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("visual")
    v = Visual(rate=30)
    v.run()
