# -*- coding: utf-8 -*-
import lcm
import sys
import os
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from localization_lcmt import localization_lcmt
from ehcd_rbctr import  EnhancedRobotCtrl

import toml
import copy
import math
import cv2
import numpy as np
from scipy.optimize import minimize
from file_send_lcmt import file_send_lcmt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

# 自定义步态发送数据的结构
robot_cmd = {
    'mode': 0, 'gait_id': 0, 'contact': 0, 'life_count': 0,
    'vel_des': [0.0, 0.0, 0.0],
    'rpy_des': [0.0, 0.0, 0.0],
    'pos_des': [0.0, 0.0, 0.0],
    'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point': [0.0, 0.0, 0.0],
    'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'step_height': [0.0, 0.0],
    'value': 0, 'duration': 0
}

# 双相机图像数据（全局变量）
left_camera_image = None  # 左相机图像（检测左黄线）
right_camera_image = None  # 右相机图像（检测右黄线）
left_image_ts = 0  # 左图像时间戳（系统时间）
right_image_ts = 0  # 右图像时间戳（系统时间）
image_lock = Lock()  # 全局图像锁

class LeftCameraSubscriber(Node):
    """左相机订阅器（专注左黄线检测）"""
    def __init__(self):
        # 调用父类Node的构造函数，传入节点名称
        super().__init__('left_camera_subscriber')
        self.bridge = CvBridge()
        # 调整QoS配置（确保与发布者兼容）
        qos_profile = QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Image,
            '/image_left',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        global left_camera_image, left_image_ts, image_lock
        with image_lock:
            try:
                left_camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                left_image_ts = time.time()
                #print(f"[左相机回调] 收到新图像，时间戳: {left_image_ts:.6f}")
            except Exception as e:
                print(f"左相机图像处理错误: {e}")


class RightCameraSubscriber(Node):
    """右相机订阅器（专注右黄线检测）"""
    def __init__(self):
        # 调用父类Node的构造函数，传入节点名称
        super().__init__('right_camera_subscriber')
        self.bridge = CvBridge()
        # 调整QoS配置（确保与发布者兼容）
        qos_profile = QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Image,
            '/image_right',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        global right_camera_image, right_image_ts, image_lock
        with image_lock:
            try:
                right_camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                right_image_ts = time.time()
                #print(f"[右相机回调] 收到新图像，时间戳: {right_image_ts:.6f}")
            except Exception as e:
                print(f"右相机图像处理错误: {e}")



class LineControl(object):
    def __init__(self):
        # 线程与通信初始化
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.odo_thread = Thread(target=self.rec_responce_o)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")

        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.odo_msg = localization_lcmt()
        self.send_lock = Lock()

        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1

        # 里程计与步态通信
        self.lc_o = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.usergait_msg = file_send_lcmt()

        # 双相机初始化
        rclpy.init(args=None)
        self.left_cam_sub = LeftCameraSubscriber()
        self.right_cam_sub = RightCameraSubscriber()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.left_cam_sub)
        self.executor.add_node(self.right_cam_sub)
        self.camera_executor_thread = Thread(target=self.executor_spin, daemon=True)

        # 相机标定参数
        self.left_cam_matrix = np.array([[615.2, 0, 320.5],
                                         [0, 614.8, 240.3],
                                         [0, 0, 1]], dtype=np.float32)
        self.right_cam_matrix = np.array([[616.1, 0, 319.8],
                                          [0, 615.5, 239.7],
                                          [0, 0, 1]], dtype=np.float32)
        self.left_dist_coeffs = np.array([-0.031, 0.082, 0.001, -0.002, -0.065], dtype=np.float32)
        self.right_dist_coeffs = np.array([-0.029, 0.078, 0.002, -0.001, -0.059], dtype=np.float32)

        # 相机位置参数（相对于机器人中心，单位：米）
        self.left_cam_pose = np.array([-0.15, 0.2, 0.2])  # 左相机位置 (x,y,z)
        self.right_cam_pose = np.array([0.15, 0.2, 0.2])  # 右相机位置 (x,y,z)
        self.cam_rotation = np.array([0, math.radians(10), 0])  # 相机俯仰角10度

        # 黄线检测参数
        self.yellow_lower = np.array([20, 80, 80])  # 放宽HSV下限，适应更多黄线场景
        self.yellow_upper = np.array([45, 255, 255])  # 放宽HSV上限
        self.min_contour_area = 50  # 降低最小轮廓面积阈值
        self.roi_ratio = 0.8  # ROI区域比例（保留图像下方80%）
        self.bottom_threshold_ratio = 0.8  # 底部10%区域的阈值（90%高度以上）
        self.min_valid_points = 2  # 降低有效点数量要求

        # 控制参数
        self.adjustment_distance = 0.15  # 每次调整距离(米)
        self.adjustment_vel = 0.1  # 调整速度(米/秒)

        # 检测状态变量
        self.left_line_data = None  # 左黄线数据
        self.right_line_data = None  # 右黄线数据
        self.lost_left_count = 0
        self.lost_right_count = 0
        self.max_lost_count = 15  # 提高连续丢失容忍次数
        self.debug_mode = True
        self.last_left_ts = 0  # 上次处理的左图像时间戳
        self.last_right_ts = 0  # 上次处理的右图像时间戳

    def executor_spin(self):
        """修复相机订阅线程：持续运行并处理异常，确保不终止"""
        while self.runing:  # 仅在程序运行时执行
            try:
                # 单次spin避免阻塞，超时1秒确保能响应退出信号
                self.executor.spin()
                time.sleep(0.01)  # 微小延迟，降低CPU占用
            except Exception as e:
                print(f"相机执行器异常: {e}，尝试重启...")
                # 重建执行器（若崩溃）
                self.executor.shutdown()
                self.executor = MultiThreadedExecutor(num_threads=2)
                self.executor.add_node(self.left_cam_sub)
                self.executor.add_node(self.right_cam_sub)
                time.sleep(0.5)  # 等待重启稳定
        # 程序退出时清理
        self.executor.shutdown()
        self.left_cam_sub.destroy_node()
        self.right_cam_sub.destroy_node()

    def run(self):
        try:
            self.lc_r.subscribe("robot_control_response", self.msg_handler)
            self.lc_o.subscribe("global_to_robot", self.msg_handler_o)
            self.send_thread.start()
            self.rec_thread.start()
            self.odo_thread.start()
            self.camera_executor_thread.start()

            # 检查相机话题是否存在（提前发现配置错误）
           # 检查相机话题是否存在（兼容所有ROS 2版本）
            temp_node = Node("temp_topic_checker")
            topics = temp_node.get_topic_names_and_types()  # 使用节点的方法获取话题
            has_left = any("/image_left" in name for name, _ in topics)
            has_right = any("/image_right" in name for name, _ in topics)
            temp_node.destroy_node()
            if not has_left or not has_right:
                missing = []
                if not has_left:
                    missing.append("/image_left")
                if not has_right:
                    missing.append("/image_right")
                raise Exception(f"相机话题不存在: {missing}，请检查发布者是否运行")

            # 等待相机初始化（获取第一帧图像）
            start_time = time.time()
            while (left_camera_image is None or right_camera_image is None) and time.time() - start_time < 10:
                time.sleep(0.1)

            if left_camera_image is None or right_camera_image is None:
                raise Exception("相机初始化失败，10秒内未获取到图像")

            if self.debug_mode:
                print("[系统] 初始化完成，开始基于图像底部10%的黄线监测")
        except Exception as e:
            print(f"运行初始化错误: {e}")
            raise

    def msg_handler(self, channel, data):
        try:
            self.rec_msg = robot_control_response_lcmt().decode(data)
            if self.rec_msg.order_process_bar >= 95:
                self.mode_ok = self.rec_msg.mode
                self.gait_ok = self.rec_msg.gait_id
            else:
                self.mode_ok = 0
        except Exception as e:
            print(f"消息处理错误: {e}")

    def msg_handler_o(self, channel, data):
        try:
            self.odo_msg = localization_lcmt().decode(data)
        except Exception as e:
            print(f"里程计消息处理错误: {e}")

    def rec_responce(self):
        while self.runing:
            try:
                self.lc_r.handle()
                time.sleep(0.002)
            except Exception as e:
                print(f"响应接收错误: {e}")
                time.sleep(0.1)

    def rec_responce_o(self):
        while self.runing:
            try:
                self.lc_o.handle()
                time.sleep(0.002)
            except Exception as e:
                print(f"里程计接收错误: {e}")
                time.sleep(0.1)

    def undistort_image(self, image, is_left_cam):
        """图像畸变矫正"""
        if image is None:
            return None

        try:
            if is_left_cam:
                matrix = self.left_cam_matrix
                dist = self.left_dist_coeffs
            else:
                matrix = self.right_cam_matrix
                dist = self.right_dist_coeffs
            h, w = image.shape[:2]
            new_cam_mat, roi = cv2.getOptimalNewCameraMatrix(matrix, dist, (w, h), 1, (w, h))
            undistorted = cv2.undistort(image, matrix, dist, None, new_cam_mat)
            x, y, w_roi, h_roi = roi
            return undistorted[y:y+h_roi, x:x+w_roi]
        except Exception as e:
            print(f"图像畸变矫正错误: {e}")
            return image

    def pixel_to_robot_3d(self, u, v, is_left_cam):
        """保留坐标转换功能（仅用于调试）"""
        try:
            if is_left_cam:
                cam_matrix = self.left_cam_matrix
                cam_pose = self.left_cam_pose
            else:
                cam_matrix = self.right_cam_matrix
                cam_pose = self.right_cam_pose

            fx, fy = cam_matrix[0, 0], cam_matrix[1, 1]
            cx, cy = cam_matrix[0, 2], cam_matrix[1, 2]

            x_cam_norm = (u - cx) / fx
            y_cam_norm = (v - cy) / fy

            # 相机旋转矩阵
            rx, ry, rz = self.cam_rotation
            rot_x = np.array([[1, 0, 0],
                             [0, math.cos(rx), -math.sin(rx)],
                             [0, math.sin(rx), math.cos(rx)]])
            rot_y = np.array([[math.cos(ry), 0, math.sin(ry)],
                             [0, 1, 0],
                             [-math.sin(ry), 0, math.cos(ry)]])
            rot_z = np.array([[math.cos(rz), -math.sin(rz), 0],
                             [math.sin(rz), math.cos(rz), 0],
                             [0, 0, 1]])
            rot_total = rot_z @ rot_y @ rot_x

            # 计算地面交点
            ray_dir_cam = np.array([x_cam_norm, y_cam_norm, 1.0])
            ray_dir_robot = rot_total @ ray_dir_cam

            t = -cam_pose[2] / ray_dir_robot[2]
            x_robot = cam_pose[0] + t * ray_dir_robot[0]
            y_robot = cam_pose[1] + t * ray_dir_robot[1]
            z_robot = 0.0

            return (x_robot, y_robot, z_robot)
        except Exception as e:
            print(f"像素坐标转换错误: {e}")
            return (0, 0, 0)

    def detect_line_features(self, image, is_left_cam, image_ts):
        """检测黄线特征，重点提取最近点的像素坐标"""
        cam_type = "左" if is_left_cam else "右"
        if image is None:
            if self.debug_mode:
                print(f"[{cam_type}相机] 无图像数据")
            return None

        try:
            # 预处理：畸变矫正 + 裁剪ROI（保留图像下方80%）
            undistorted = self.undistort_image(image, is_left_cam)
            if undistorted is None:
                return None

            h, w = undistorted.shape[:2]
            roi_y_start = int(h * (1 - self.roi_ratio))  # ROI从图像20%高度开始
            roi = undistorted[roi_y_start:h, :]
            roi_h, roi_w = roi.shape[:2]

            # 黄线检测（HSV阈值过滤）
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)

            # 形态学滤波（去除噪声）
            kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_small)  # 闭合小空洞
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)   # 去除小噪点

            # 轮廓检测
            try:
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            except ValueError:
                _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                if self.debug_mode:
                    print(f"[{cam_type}相机] 未检测到黄线轮廓")
                return None

            # 筛选最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(max_contour)
            if contour_area < self.min_contour_area:
                if self.debug_mode:
                    print(f"[{cam_type}相机] 黄线轮廓面积过小（{contour_area:.1f} < {self.min_contour_area}）")
                return None

            # 提取轮廓点并转换为原图坐标
            contour_pts = max_contour.squeeze()
            if len(contour_pts.shape) == 1:
                contour_pts = np.expand_dims(contour_pts, axis=0)
            full_pts = np.zeros_like(contour_pts, dtype=np.float32)
            full_pts[:, 0] = contour_pts[:, 0]
            full_pts[:, 1] = contour_pts[:, 1] + roi_y_start  # 转换回原图的y坐标

            # 筛选有效点
            valid_pts = []
            for (u, v) in full_pts:
                if 0 <= u < w and 0 <= v < h:
                    valid_pts.append((u, v))

            if len(valid_pts) < self.min_valid_points:
                if self.debug_mode:
                    print(f"[{cam_type}相机] 有效黄线点不足（{len(valid_pts)} < {self.min_valid_points}）")
                return None

            # 找到最近的点（y坐标最大的点，即图像中最靠下的点）
            max_v_idx = np.argmax([p[1] for p in valid_pts])
            closest_pixel = valid_pts[max_v_idx]

            # 调试可视化
            if self.debug_mode:
                debug_img = undistorted.copy()
                cv2.drawContours(debug_img[roi_y_start:h, :], [max_contour], -1, (0, 255, 0), 2)
                cv2.line(debug_img, (0, roi_y_start), (w, roi_y_start), (255, 0, 0), 2)  # ROI上边界
                bottom_line_y = int(h * self.bottom_threshold_ratio)
                cv2.line(debug_img, (0, bottom_line_y), (w, bottom_line_y), (0, 0, 255), 2)  # 底部10%红线
                cv2.circle(debug_img, (int(closest_pixel[0]), int(closest_pixel[1])), 5, (0, 255, 255), -1)
                is_in_bottom = closest_pixel[1] >= bottom_line_y
                cv2.putText(debug_img, f"底部10%: {is_in_bottom}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(f"{cam_type}相机黄线检测", debug_img)
                cv2.waitKey(1)

            return {
                'image_shape': (w, h),
                'closest_pixel': closest_pixel,
                'timestamp': image_ts
            }
        except Exception as e:
            print(f"[{cam_type}相机] 黄线检测错误: {e}")
            return None

    def update_line_data(self):
        """更新左右黄线数据（核心优化：强制更新机制）"""
        try:
            with image_lock:
                left_img = None
                right_img = None
                new_left = False
                new_right = False
                current_time = time.time()

                # 左相机：允许时间戳相等（极端情况），3秒无新数据则强制更新
                if left_camera_image is not None and left_image_ts > 0:
                    force_update = (current_time - self.last_left_ts) > 3.0  # 3秒无新数据则强制
                    if left_image_ts > self.last_left_ts or force_update:
                        left_img = left_camera_image.copy()
                        left_ts = left_image_ts
                        new_left = True
                        if force_update:
                            print(f"[左相机] 3秒无新数据，强制更新")

                # 右相机同理
                if right_camera_image is not None and right_image_ts > 0:
                    force_update = (current_time - self.last_right_ts) > 3.0
                    if right_image_ts > self.last_right_ts or force_update:
                        right_img = right_camera_image.copy()
                        right_ts = right_image_ts
                        new_right = True
                        if force_update:
                            print(f"[右相机] 3秒无新数据，强制更新")

            # 处理左相机新图像
            left_data = None
            if new_left and left_img is not None:
                left_data = self.detect_line_features(left_img, is_left_cam=True, image_ts=left_ts)
                if left_data is not None:
                    self.left_line_data = left_data
                    self.lost_left_count = 0
                    self.last_left_ts = left_ts
                    print(f"[左相机] 数据更新成功，时间戳: {left_ts:.6f}")
                else:
                    self.lost_left_count += 1
                    print(f"[左相机] 检测失败，连续丢失次数: {self.lost_left_count}")
                    if self.lost_left_count > self.max_lost_count:
                        self.left_line_data = None

            # 处理右相机新图像
            right_data = None
            if new_right and right_img is not None:
                right_data = self.detect_line_features(right_img, is_left_cam=False, image_ts=right_ts)
                if right_data is not None:
                    self.right_line_data = right_data
                    self.lost_right_count = 0
                    self.last_right_ts = right_ts
                    print(f"[右相机] 数据更新成功，时间戳: {right_ts:.6f}")
                else:
                    self.lost_right_count += 1
                    print(f"[右相机] 检测失败，连续丢失次数: {self.lost_right_count}")
                    if self.lost_right_count > self.max_lost_count:
                        self.right_line_data = None

        except Exception as e:
            print(f"更新黄线数据错误: {e}")

    def is_stepping_on_line(self, line_data):
        """判断黄线是否出现在图像高度20%处"""
        if not line_data:
            return False

        h = line_data['image_shape'][1]  # 图像高度
        v = line_data['closest_pixel'][1]  # 黄线最近点的y坐标（像素）
        target_line_y = int(h * 0.2)  # 20%高度处的y坐标

        # 允许一定的容差范围（例如±5像素）
        tolerance = 5
        return abs(v - target_line_y) <= tolerance


    def check_line_position(self):
        """检查黄线位置（仅基于图像底部10%判断）"""
        self.update_line_data()

        if self.left_line_data is None or self.right_line_data is None:
            print("黄线数据不完整，无法判断位置")
            return False, ""

        left_stepping = self.is_stepping_on_line(self.left_line_data)
        right_stepping = self.is_stepping_on_line(self.right_line_data)

        print(f"左黄线底部10%: {left_stepping}, 右黄线底部10%: {right_stepping}")

        if left_stepping and not right_stepping:
            print("左黄线出现在底部10%，需要向右调整")
            return True, "right"
        elif right_stepping and not left_stepping:
            print("右黄线出现在底部10%，需要向左调整")
            return True, "left"
        elif left_stepping and right_stepping:
            print("左右黄线均出现在底部10%，无法安全调整")
            return False, ""
        else:
            return False, ""

    def adjust_position(self, msg, direction):
        """调整机器人位置"""
        if direction not in ["left", "right"]:
            print("无效的调整方向")
            return False

        try:
            adjust_time = self.adjustment_distance / self.adjustment_vel

            msg.mode = 11
            msg.gait_id = 26
            lateral_vel = -self.adjustment_vel if direction == "right" else self.adjustment_vel
            msg.vel_des = [0.0, lateral_vel, 0.0]  # [前进, 横向, 转向]
            msg.duration = int(adjust_time * 1000)
            msg.life_count += 1

            print(f"执行{direction}调整，距离: {self.adjustment_distance}m, 速度: {lateral_vel:.2f}m/s")
            self.Send_cmd(msg)

            success = self.Wait_finish(11, 26)
            if not success:
                print(f"{direction}调整超时")
            else:
                print(f"{direction}调整完成")

            return success
        except Exception as e:
            print(f"位置调整错误: {e}")
            return False

    def perform_line_detection_and_adjustment(self, msg):
        """连续调整多次直到满足条件（确保每次更新检测状态）"""
        max_adjustments = 1
        adjustment_count = 0
        prev_left_state = False
        prev_right_state = False
        stall_count = 0

        print("\n开始连续黄线调整流程（最大调整次数：{}）".format(max_adjustments))

        while adjustment_count < max_adjustments:
            # 强制机器人停止并进入站立状态
            print("\n===== 准备检测：强制机器人停止 =====")
            msg.mode = 12  # Recovery stand模式（站立）
            msg.gait_id = 0
            msg.vel_des = [0.0, 0.0, 0.0]
            msg.life_count += 1
            self.Send_cmd(msg)

            # 等待停止完成
            stop_success = self.Wait_finish(12, 0)
            if not stop_success:
                print("警告：机器人未成功停止，尝试再次发送停止指令")
                msg.life_count += 1
                self.Send_cmd(msg)
                stop_success = self.Wait_finish(12, 0)
                if not stop_success:
                    print("错误：机器人无法停止，终止调整流程")
                    break

            # 停止后等待2秒，确保图像稳定
            # print("机器人已停止，等待2秒稳定图像...")
            # time.sleep(2)

            # 强制更新图像数据（3次确保最新）
            print("强制更新最新黄线数据...")
            for _ in range(3):
                self.update_line_data()
                time.sleep(0.1)

            # 显式计算最新状态（确保每次更新）
            left_stepping = False
            if self.left_line_data is not None:
                left_stepping = self.is_stepping_on_line(self.left_line_data)
            else:
                print("警告：左黄线数据为空，设为未检测到")

            right_stepping = False
            if self.right_line_data is not None:
                right_stepping = self.is_stepping_on_line(self.right_line_data)
            else:
                print("警告：右黄线数据为空，设为未检测到")

            # 打印当前状态
            print(f"\n调整次数：{adjustment_count + 1}/{max_adjustments}")
            print(f"当前状态 - 左黄线底部10%: {left_stepping}, 右黄线底部10%: {right_stepping}")
            print(f"数据时间戳 - 左: {self.left_line_data['timestamp'] if self.left_line_data else '无'}")
            print(f"数据时间戳 - 右: {self.right_line_data['timestamp'] if self.right_line_data else '无'}")

            # 检查终止条件
            if (not left_stepping and not right_stepping) or (left_stepping and right_stepping):
                print("\n满足终止条件，停止调整")
                break

            # 检查停滞状态
            if left_stepping == prev_left_state and right_stepping == prev_right_state:
                stall_count += 1
                if stall_count >= 2:
                    print("\n检测到调整停滞，停止调整")
                    break
            else:
                stall_count = 0
                prev_left_state = left_stepping
                prev_right_state = right_stepping

            # 判断是否需要调整
            adjustment_needed = False
            direction = ""
            if left_stepping and not right_stepping:
                adjustment_needed = True
                direction = "right"
            elif right_stepping and not left_stepping:
                adjustment_needed = True
                direction = "left"

            # 执行调整
            if adjustment_needed:
                print(f"检测到黄线出现在底部10%区域，向{direction}方向调整")

                # 动态调整步长
                current_adjustment = self.adjustment_distance if adjustment_count < 2 else self.adjustment_distance * 0.5

                # 执行调整动作
                success = self.adjust_position(msg, direction)
                if not success:
                    print("调整执行失败，停止调整流程")
                    break

                adjustment_count += 1
                # 调整后等待图像更新
                time.sleep(1.5)
            else:
                print("无需调整，退出循环")
                break

        # 最终状态确认
        self.update_line_data()
        final_left = self.is_stepping_on_line(self.left_line_data) if self.left_line_data else False
        final_right = self.is_stepping_on_line(self.right_line_data) if self.right_line_data else False
        print(f"\n连续调整结束，最终状态 - 左黄线底部10%: {final_left}, 右黄线底部10%: {final_right}")

        # 确保机器人最终处于停止状态
        msg.mode = 12
        msg.gait_id = 0
        msg.vel_des = [0.0, 0.0, 0.0]
        msg.life_count += 1
        self.Send_cmd(msg)
        self.Wait_finish(12, 0)

    def Wait_finish(self, mode, gait_id):
        """等待指令完成"""
        try:
            count = 0
            timeout = 8000  # 超时时间8秒
            while self.runing and count < timeout:
                if self.mode_ok == mode and self.gait_ok == gait_id:
                    if self.debug_mode:
                        print(f"[指令完成] 模式: {mode}, 步态ID: {gait_id}")
                    return True
                time.sleep(0.001)
                count += 1
            if self.debug_mode:
                print(f"[指令超时] 模式: {mode}, 步态ID: {gait_id}")
            return False
        except Exception as e:
            print(f"等待完成错误: {e}")
            return False

    def send_publish(self):
        """发送指令循环"""
        while self.runing:
            try:
                self.send_lock.acquire()
                if self.delay_cnt > 20:
                    self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                    self.delay_cnt = 0
                self.delay_cnt += 1
                self.send_lock.release()
                time.sleep(0.005)
            except Exception as e:
                print(f"发送发布错误: {e}")
                self.send_lock.release()
                time.sleep(0.1)

    def Send_cmd(self, msg):
        """发送单条指令"""
        try:
            self.send_lock.acquire()
            self.delay_cnt = 50
            self.cmd_msg = msg
            if self.debug_mode:
                print(f"[发送指令] 生命周期: {msg.life_count}")
            self.send_lock.release()
        except Exception as e:
            print(f"发送指令错误: {e}")
            self.send_lock.release()

    def quit(self):
        """退出并释放资源"""
        try:
            global left_camera_image, right_camera_image
            left_camera_image = None
            right_camera_image = None

            self.runing = 0
            if hasattr(self, 'rec_thread') and self.rec_thread.is_alive():
                print("1")
                self.rec_thread.join(timeout=1.0)
            if hasattr(self, 'send_thread') and self.send_thread.is_alive():
                print("2")
                self.send_thread.join(timeout=1.0)
            if hasattr(self, 'odo_thread') and self.odo_thread.is_alive():
                print("3")
                self.odo_thread.join(timeout=1.0)
            if hasattr(self, 'camera_executor_thread') and self.camera_executor_thread.is_alive():
                print("4")
                self.camera_executor_thread.join(timeout=1.0)

            rclpy.shutdown()
            cv2.destroyAllWindows()
            if self.debug_mode:
                print("[系统] 退出并释放资源")
        except Exception as e:
            print(f"退出错误: {e}")


