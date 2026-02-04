import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from robot_controller import RobotController
from robot_control import Robot_Ctrl

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt


class ImageSubscriber(Node):
    def __init__(self, robot_ctrl):
        super().__init__('image_subscriber')
        self.robot_ctrl = robot_ctrl
        self.bridge = CvBridge()

        # 检测开关
        self.enable_detection = False
        self.enable_light_detection = False  # 黄色灯光点的独立检测开关
        self.enable_arrow_detection = False  # 新增：绿色箭头检测开关

        # 控制参数
        self.last_cmd_time = 0
        self.cmd_cooldown = 5.0
        self.command_sent = False

        # 新增调整参数
        self.adjustment_enabled = False  # 是否允许位置调整
        self.last_adjust_time = 0
        self.adjust_cooldown = 5.0  # 调整指令冷却时间

        # 颜色检测参数
        self.lower_yellow = np.array([25, 110, 245])
        self.upper_yellow = np.array([35, 120, 255])

        # 新增：绿色箭头的HSV阈值
        self.lower_green = np.array([40, 40, 40])  # 绿色范围下限
        self.upper_green = np.array([80, 255, 255])  # 绿色范围上限

        self.detection_point_yellow_light = (160, 80)

        # 四个检测点坐标 (x,y)
        self.detection_points = [
            (1, 179),  # 左下
            (10, 179),  # 左中下
            (309, 179),  # 右中下
            (319, 179)  # 右下
        ]

        # 新增：箭头检测点坐标 (x,y)
        self.detection_arrow = [
            (150, 75),  # 左箭头点1
            (150, 85),  # 左箭头点2
            (160, 75),  # 右箭头点1
            (160, 55)  # 右箭头点2
        ]

        # QoS设置
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            qos
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            debug_img = cv_image.copy()

            # 绘制所有检测点
            for x, y in self.detection_points:
                cv2.circle(debug_img, (x, y), 3, (0, 0, 255), -1)  # 底部点红色
            # 绘制灯光检测点
            cv2.circle(debug_img, self.detection_point_yellow_light, 5, (255, 0, 0), -1)  # 灯光点蓝色

            # 绘制箭头检测点
            for i, (x, y) in enumerate(self.detection_arrow):
                color = (0, 255, 0) if i < 2 else (255, 255, 0)  # 左箭头点绿色，右箭头点青色
                cv2.circle(debug_img, (x, y), 3, color, -1)

            # 黄色灯光检测逻辑（独立控制）
            light_detected = False

            if self.enable_light_detection:
                lx, ly = self.detection_point_yellow_light
                pixel_hsv = hsv[ly, lx]
                light_detected = (self.lower_yellow[0] <= pixel_hsv[0] <= self.upper_yellow[0] and
                                  self.lower_yellow[1] <= pixel_hsv[1] <= self.upper_yellow[1] and
                                  self.lower_yellow[2] <= pixel_hsv[2] <= self.upper_yellow[2])

                # 标记灯光检测点状态
                light_color = (0, 255, 0) if light_detected else (255, 0, 0)
                cv2.circle(debug_img, (lx, ly), 7, light_color, 1)

                # 灯光检测指令逻辑
                current_time = time.time()
                if light_detected and not self.command_sent:
                    if current_time - self.last_cmd_time > self.cmd_cooldown:
                        self.get_logger().info("检测到黄色灯光，发送执行指令")
                        self.robot_ctrl.send_move_command(
                            mode=11, gait_id=27, vel_des=[0, 0, 0], duration=2000
                        )
                        self.command_sent = True
                        self.last_cmd_time = current_time
                elif not light_detected:
                    self.command_sent = False

            # 底部四点检测逻辑（原有逻辑）
            if self.enable_detection:
                # 检查各点颜色状态
                point_status = []
                for x, y in self.detection_points:
                    pixel_hsv = hsv[y, x]
                    is_yellow = (self.lower_yellow[0] <= pixel_hsv[0] <= self.upper_yellow[0] and
                                 self.lower_yellow[1] <= pixel_hsv[1] <= self.upper_yellow[1] and
                                 self.lower_yellow[2] <= pixel_hsv[2] <= self.upper_yellow[2])
                    point_status.append(is_yellow)

                yellow_count = sum(point_status)
                enough_yellow = yellow_count >= 3  # 修改为至少3个点

                # 分析左右两侧状态
                left_status = point_status[0] or point_status[1]  # 左侧两点
                right_status = point_status[2] or point_status[3]  # 右侧两点

                current_time = time.time()

                # 位置调整逻辑
                if self.adjustment_enabled and not enough_yellow:
                    if current_time - self.last_adjust_time > self.adjust_cooldown:
                        if not left_status and right_status:
                            # 左侧不黄，向右微调
                            self.get_logger().info("左侧不黄，向右调整")
                            self.robot_ctrl.send_move_command(
                                mode=11, gait_id=27, vel_des=[0, -0.1, 0], duration=500
                            )
                            self.last_adjust_time = current_time
                        elif left_status and not right_status:
                            # 右侧不黄，向左微调
                            self.get_logger().info("右侧不黄，向左调整")
                            self.robot_ctrl.send_move_command(
                                mode=11, gait_id=27, vel_des=[0, 0.1, 0], duration=500
                            )
                            self.last_adjust_time = current_time
                        elif not left_status and not right_status:
                            # 两侧都不黄，可能需要更大调整
                            self.get_logger().info("两侧都不黄，需要更大调整")
                            # 根据具体情况设计调整策略

                # 直行指令逻辑
                if enough_yellow and not self.command_sent:
                    if current_time - self.last_cmd_time > self.cmd_cooldown:
                        self.get_logger().info("所有检测点均为黄色，发送直行指令")
                        self.robot_ctrl.send_move_command(
                            mode=11, gait_id=27, vel_des=[0, 0, 0], duration=2000
                        )
                        self.robot_ctrl.Wait_finish(11, 27)
                        self.command_sent = True
                        self.last_cmd_time = current_time
                elif not enough_yellow:
                    self.command_sent = False

                status_text = f"检测中: {yellow_count}/4 黄色 | 左:{'黄' if left_status else '非黄'} 右:{'黄' if right_status else '非黄'}"
            else:
                status_text = "检测已禁用"

            # 新增：绿色箭头检测逻辑
            arrow_detected = False
            arrow_direction = None
            if self.enable_arrow_detection:
                # 检查箭头点颜色状态
                arrow_status = []
                for x, y in self.detection_arrow:
                    pixel_hsv = hsv[y, x]
                    is_green = (self.lower_green[0] <= pixel_hsv[0] <= self.upper_green[0] and
                                self.lower_green[1] <= pixel_hsv[1] <= self.upper_green[1] and
                                self.lower_green[2] <= pixel_hsv[2] <= self.upper_green[2])
                    arrow_status.append(is_green)

                # 判断方向
                left_arrow = arrow_status[0] and arrow_status[1]  # 左箭头两个点
                right_arrow = arrow_status[2] and arrow_status[3]  # 右箭头两个点

                current_time = time.time()

                if left_arrow and not right_arrow and not self.command_sent:
                    if current_time - self.last_cmd_time > self.cmd_cooldown:
                        self.get_logger().info("检测到左箭头，发送左转指令")
                        self.robot_ctrl.send_move_command(
                            mode=11, gait_id=27, vel_des=[0, 0.1, 0], duration=1000  # 左转
                        )
                        self.command_sent = True
                        self.last_cmd_time = current_time
                        arrow_direction = "左"

                elif right_arrow and not left_arrow and not self.command_sent:
                    if current_time - self.last_cmd_time > self.cmd_cooldown:
                        self.get_logger().info("检测到右箭头，发送右转指令")
                        self.robot_ctrl.send_move_command(
                            mode=11, gait_id=27, vel_des=[0, -0.1, 0], duration=1000  # 右转
                        )
                        self.command_sent = True
                        self.last_cmd_time = current_time
                        arrow_direction = "右"

                elif not left_arrow and not right_arrow:
                    self.command_sent = False
                    arrow_direction = None

                # 在图像上标记箭头状态
                for i, (x, y) in enumerate(self.detection_arrow):
                    color = (0, 255, 0) if arrow_status[i] else (100, 100, 100)
                    cv2.circle(debug_img, (x, y), 5, color, 1)

            # 显示状态
            status_text = f"灯光检测: {'开启' if self.enable_light_detection else '关闭'}"
            if self.enable_light_detection:
                status_text += f" | 灯光:{'检测到' if light_detected else '未检测'}"

            if self.enable_arrow_detection:
                status_text += f" | 箭头:{arrow_direction if arrow_direction else '未检测'}"

            cv2.putText(debug_img, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow('Detection', debug_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"处理错误: {str(e)}")

    def set_detection_enable(self, enable, allow_adjustment=False):
        """设置是否启用颜色检测和调整"""
        self.enable_detection = enable
        self.adjustment_enabled = allow_adjustment
        self.get_logger().info(
            f"颜色检测已{'启用' if enable else '禁用'}, 调整{'开启' if allow_adjustment else '关闭'}")
        self.command_sent = False
        self.last_adjust_time = 0

    def set_light_detection_enable(self, enable):
        """独立设置黄色灯光检测"""
        self.enable_light_detection = enable
        self.get_logger().info(f"黄色灯光检测已{'启用' if enable else '禁用'}")
        self.command_sent = False

    def set_arrow_detection_enable(self, enable):
        """设置绿色箭头检测开关"""
        self.enable_arrow_detection = enable
        self.get_logger().info(f"绿色箭头检测已{'启用' if enable else '禁用'}")
        self.command_sent = False