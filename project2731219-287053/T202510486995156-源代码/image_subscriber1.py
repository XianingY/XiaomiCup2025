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
        self.enable_edge_detection = False   # S弯边缘检测开关

        # 控制参数
        self.last_cmd_time = 0
        self.cmd_cooldown = 5.0
        self.command_sent = False

        # 新增调整参数
        self.adjustment_enabled = False  # 是否允许位置调整
        self.last_adjust_time = 0
        self.adjust_cooldown = 5.0  # 调整指令冷却时间

        # 边缘检测调整参数
        self.last_edge_adjust_time = 0
        self.edge_adjust_cooldown = 2.0  # 边缘调整冷却时间（更短，反应更快）

        # 颜色检测参数
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
        # 边缘检测颜色范围（检测非黄色区域作为边缘）
        self.lower_edge = np.array([0, 0, 0])      # 黑色/灰色下限
        self.upper_edge = np.array([180, 50, 50])  # 非黄色上限

        self.detection_point_yellow_light = (160, 80)
        
        # 四个检测点坐标 (x,y)
        self.detection_points = [
            (1, 179),  # 左下
            (10, 179),  # 左中下
            (309, 179),  # 右中下
            (319, 179)  # 右下
        ]

        # S弯边缘检测点 - 在机器人前方和两侧设置检测点
        self.edge_detection_points = {
            'left_front': [(20, 120), (30, 130), (40, 140)],      # 左前方检测点
            'right_front': [(280, 120), (290, 130), (300, 140)],  # 右前方检测点
            'left_side': [(5, 150), (10, 160), (15, 170)],        # 左侧检测点
            'right_side': [(305, 150), (310, 160), (315, 170)],   # 右侧检测点
        }

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

    def detect_edge_danger(self, hsv):
        """检测边缘危险情况"""
        edge_status = {
            'left_danger': False,
            'right_danger': False,
            'danger_level': 0  # 0-无危险, 1-轻微, 2-中等, 3-严重
        }
        
        # 检测左侧边缘
        left_edge_count = 0
        for point_group in ['left_front', 'left_side']:
            for x, y in self.edge_detection_points[point_group]:
                if 0 <= y < hsv.shape[0] and 0 <= x < hsv.shape[1]:
                    pixel_hsv = hsv[y, x]
                    # 检测是否为非黄色（边缘）
                    is_edge = not (self.lower_yellow[0] <= pixel_hsv[0] <= self.upper_yellow[0] and
                                  self.lower_yellow[1] <= pixel_hsv[1] <= self.upper_yellow[1] and
                                  self.lower_yellow[2] <= pixel_hsv[2] <= self.upper_yellow[2])
                    if is_edge:
                        left_edge_count += 1
        
        # 检测右侧边缘
        right_edge_count = 0
        for point_group in ['right_front', 'right_side']:
            for x, y in self.edge_detection_points[point_group]:
                if 0 <= y < hsv.shape[0] and 0 <= x < hsv.shape[1]:
                    pixel_hsv = hsv[y, x]
                    # 检测是否为非黄色（边缘）
                    is_edge = not (self.lower_yellow[0] <= pixel_hsv[0] <= self.upper_yellow[0] and
                                  self.lower_yellow[1] <= pixel_hsv[1] <= self.upper_yellow[1] and
                                  self.lower_yellow[2] <= pixel_hsv[2] <= self.upper_yellow[2])
                    if is_edge:
                        right_edge_count += 1
        
        # 判断危险程度
        left_total_points = len(self.edge_detection_points['left_front']) + len(self.edge_detection_points['left_side'])
        right_total_points = len(self.edge_detection_points['right_front']) + len(self.edge_detection_points['right_side'])
        
        left_danger_ratio = left_edge_count / left_total_points
        right_danger_ratio = right_edge_count / right_total_points
        
        # 设置危险阈值
        if left_danger_ratio >= 0.5:  # 50%以上的点检测到边缘
            edge_status['left_danger'] = True
            edge_status['danger_level'] = max(edge_status['danger_level'], 
                                            3 if left_danger_ratio >= 0.8 else 
                                            2 if left_danger_ratio >= 0.7 else 1)
        
        if right_danger_ratio >= 0.5:
            edge_status['right_danger'] = True
            edge_status['danger_level'] = max(edge_status['danger_level'], 
                                            3 if right_danger_ratio >= 0.8 else 
                                            2 if right_danger_ratio >= 0.7 else 1)
        
        return edge_status, left_edge_count, right_edge_count

    def execute_edge_avoidance(self, edge_status):
        """执行边缘避让动作"""
        current_time = time.time()
        
        if current_time - self.last_edge_adjust_time < self.edge_adjust_cooldown:
            return False
        
        # 根据危险等级和位置调整速度和持续时间
        danger_level = edge_status['danger_level']
        
        # 调整参数根据危险等级
        if danger_level == 1:  # 轻微危险
            lateral_speed = 0.15
            duration = 300
        elif danger_level == 2:  # 中等危险
            lateral_speed = 0.25
            duration = 500
        else:  # 严重危险
            lateral_speed = 0.35
            duration = 700
        
        # 决定调整方向
        if edge_status['left_danger'] and not edge_status['right_danger']:
            # 左边有危险，向右调整（负Y方向）
            self.get_logger().info(f"检测到左侧边缘危险(等级:{danger_level})，向右调整")
            self.robot_ctrl.send_move_command(
                mode=11, gait_id=27, vel_des=[0, -lateral_speed, 0], duration=duration
            )
            self.last_edge_adjust_time = current_time
            return True
            
        elif edge_status['right_danger'] and not edge_status['left_danger']:
            # 右边有危险，向左调整（正Y方向）
            self.get_logger().info(f"检测到右侧边缘危险(等级:{danger_level})，向左调整")
            self.robot_ctrl.send_move_command(
                mode=11, gait_id=27, vel_des=[0, lateral_speed, 0], duration=duration
            )
            self.last_edge_adjust_time = current_time
            return True
            
        elif edge_status['left_danger'] and edge_status['right_danger']:
            # 两边都有危险，停止或后退
            self.get_logger().warning("两侧都检测到边缘危险，执行紧急停止")
            self.robot_ctrl.send_move_command(
                mode=11, gait_id=27, vel_des=[0, 0, 0], duration=1000
            )
            self.last_edge_adjust_time = current_time
            return True
        
        return False

    def draw_edge_detection_points(self, debug_img, hsv):
        """绘制边缘检测点和状态"""
        for group_name, points in self.edge_detection_points.items():
            for x, y in points:
                if 0 <= y < hsv.shape[0] and 0 <= x < hsv.shape[1]:
                    pixel_hsv = hsv[y, x]
                    # 检测是否为非黄色（边缘）
                    is_edge = not (self.lower_yellow[0] <= pixel_hsv[0] <= self.upper_yellow[0] and
                                  self.lower_yellow[1] <= pixel_hsv[1] <= self.upper_yellow[1] and
                                  self.lower_yellow[2] <= pixel_hsv[2] <= self.upper_yellow[2])
                    
                    # 根据检测结果选择颜色
                    if is_edge:
                        color = (0, 0, 255)  # 红色表示检测到边缘
                    else:
                        color = (0, 255, 0)  # 绿色表示安全
                    
                    # 根据检测点类型选择形状
                    if 'front' in group_name:
                        cv2.circle(debug_img, (x, y), 4, color, -1)  # 前方点用实心圆
                    else:
                        cv2.circle(debug_img, (x, y), 4, color, 2)   # 侧面点用空心圆

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

            # 边缘检测逻辑
            edge_adjustment_made = False
            if self.enable_edge_detection:
                # 绘制边缘检测点
                self.draw_edge_detection_points(debug_img, hsv)
                
                # 执行边缘检测
                edge_status, left_edge_count, right_edge_count = self.detect_edge_danger(hsv)
                
                # 执行边缘避让
                edge_adjustment_made = self.execute_edge_avoidance(edge_status)
                
                # 显示边缘检测状态
                edge_text = f"边缘检测: L:{left_edge_count} R:{right_edge_count}"
                if edge_status['left_danger'] or edge_status['right_danger']:
                    edge_text += f" 危险等级:{edge_status['danger_level']}"
                cv2.putText(debug_img, edge_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

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
            if self.enable_detection and not edge_adjustment_made:  # 边缘调整时暂停底部检测
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

                # 直行指令逻辑
                if enough_yellow and not self.command_sent:
                    if current_time - self.last_cmd_time > self.cmd_cooldown:
                        self.get_logger().info("所有检测点均为黄色，发送直行指令")
                        self.command_sent = True
                        self.last_cmd_time = current_time
                elif not enough_yellow:
                    self.command_sent = False

                status_text = f"检测中: {yellow_count}/4 黄色 | 左:{'黄' if left_status else '非黄'} 右:{'黄' if right_status else '非黄'}"
            else:
                status_text = "检测已禁用"

            # 显示状态
            main_status = f"灯光:{'开' if self.enable_light_detection else '关'} 边缘:{'开' if self.enable_edge_detection else '关'}"
            if self.enable_light_detection:
                main_status += f" | 灯光:{'检测到' if light_detected else '未检测'}"
            
            cv2.putText(debug_img, main_status, (10, 30),
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

    def set_edge_detection_enable(self, enable):
        """设置S弯边缘检测"""
        self.enable_edge_detection = enable
        self.get_logger().info(f"S弯边缘检测已{'启用' if enable else '禁用'}")
        self.last_edge_adjust_time = 0

    def set_edge_detection_sensitivity(self, cooldown=2.0, danger_threshold=0.5):
        """调整边缘检测敏感度"""
        self.edge_adjust_cooldown = cooldown
        self.get_logger().info(f"边缘检测冷却时间设置为: {cooldown}秒")

    def set_edge_detection_points(self, custom_points=None):
        """自定义边缘检测点位置"""
        if custom_points:
            self.edge_detection_points = custom_points
            self.get_logger().info("已更新边缘检测点位置")