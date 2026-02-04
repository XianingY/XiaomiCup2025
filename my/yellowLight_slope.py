'''
This demo shows the communication interface of MR813 motion control board based on Lcm.
Dependency:
- robot_control_cmd_lcmt.py
- robot_control_response_lcmt.py
'''
import lcm
import sys
import os
import time

import cv2
import numpy as np
from threading import Thread, Lock, Event
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from ehcd_rbctr import EnhancedRobotCtrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from robot_control import Robot_Ctrl
from protocol.msg import AudioPlayExtend  # 导入CyberDog定义的音频消息类型

class YellowLightDetector(Node):
    def __init__(self, stop_event):
        super().__init__('yellow_light_detector')

        # 共享事件，用于通知主控制线程停止
        self.stop_event = stop_event

        # 检测使能开关（默认关闭，下坡后开启）
        self.detection_enabled = False
        self.enable_lock = Lock()  # 保护开关的线程安全

        # 黄灯实际直径（单位：米）
        self.actual_diameter = 0.20  # 20cm
        self.min_distance = 1.50    # 距离阈值

        # 当前检测到的距离
        self.current_distance = None
        self.distance_lock = Lock()

        # 创建CvBridge用于图像转换
        self.bridge = CvBridge()

        # 存储相机内参
        self.camera_matrix = None

        # 存储最新图像用于显示
        self.latest_image = None
        self.image_lock = Lock()

        # 订阅RGB图像、深度图像
        self.rgb_sub = Subscriber(self, Image, '/image_rgb')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_rect_raw')
        
        # 时间同步器
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=30,
            slop=0.2
        )
        self.ts.registerCallback(self.image_callback)

        # 订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )

        # 启动图像显示线程
        self.display_thread = Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        self.get_logger().info("黄灯检测器已启动")


    # 设置检测使能状态的方法
    def set_detection_enabled(self, enabled):
        with self.enable_lock:
            self.detection_enabled = enabled
            if enabled:
                self.get_logger().info("黄灯检测已启动")
            else:
                self.get_logger().info("黄灯检测已关闭")

    def camera_info_callback(self, msg):
        """获取相机内参矩阵"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"已获取相机内参")

    def detect_yellow_light(self, rgb_image):
        """检测图像中的黄色圆形区域"""
        # 缩小图像尺寸加快处理
        scale = 0.8
        small_img = cv2.resize(rgb_image, (0, 0), fx=scale, fy=scale)

        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(small_img, cv2.COLOR_BGR2HSV)

        # 严格的黄色HSV范围，减少误检
        lower_yellow = np.array([20, 120, 150])
        upper_yellow = np.array([30, 255, 255])

        # 创建黄色掩码
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 形态学操作去除噪声
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # 寻找轮廓
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 筛选圆形轮廓
        yellow_lights = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 80:  # 较大的面积阈值，过滤小噪声
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            # 高圆度要求
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity > 0.85:
                # 宽高比接近1（正圆）
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if 0.9 < aspect_ratio < 1.1:
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x / scale), int(y / scale))
                    radius = int(radius / scale)
                    yellow_lights.append((center, radius))

        return yellow_lights

    def calculate_distance(self, radius_pixels):
        """根据像素半径计算距离"""
        if self.camera_matrix is None or radius_pixels <= 0:
            return None

        focal_length = (self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) / 2
        distance = (self.actual_diameter * focal_length) / (2 * radius_pixels)
        return distance

    def image_callback(self, rgb_msg, depth_msg):
        """处理图像数据并检测距离"""
        try:
            # 转换图像格式
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            # 检测黄灯
            yellow_lights = self.detect_yellow_light(rgb_image)
            min_detected_distance = None

            # 处理检测结果
            for (center, radius) in yellow_lights:
                # 在图像上绘制检测到的黄灯
                cv2.circle(rgb_image, center, radius, (0, 255, 0), 2)
                cv2.circle(rgb_image, center, 3, (0, 0, 255), -1)

                # 计算距离（优先使用深度图）
                distance_depth = None
                x, y = center
                if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                    distance_depth = depth_image[y, x] / 1000.0  # 转换为米
                    if distance_depth <= 0:
                        distance_depth = None

                # 视觉计算作为备用
                distance_vision = self.calculate_distance(radius)
                distance = distance_depth if distance_depth is not None else distance_vision

                # 在图像上显示距离
                if distance:
                    cv2.putText(rgb_image, f"距离: {distance:.2f}m",
                                (center[0] - 50, center[1] - radius - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    if not min_detected_distance or distance < min_detected_distance:
                        min_detected_distance = distance

            # 显示当前最小距离
            if min_detected_distance:
                cv2.putText(rgb_image, f"最小距离: {min_detected_distance:.2f}m",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                # 如果距离小于阈值，显示警告信息
                if min_detected_distance < self.min_distance:
                    cv2.putText(rgb_image, "警告: 距离过近!",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 更新当前距离
            with self.distance_lock:
                self.current_distance = min_detected_distance

                # 如果距离小于阈值，触发停止事件
                if min_detected_distance and min_detected_distance < self.min_distance:
                    self.stop_event.set()

            # 保存最新图像用于显示
            with self.image_lock:
                self.latest_image = rgb_image.copy()

        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")

    def get_distance(self):
        """获取当前检测到的距离"""
        with self.distance_lock:
            return self.current_distance

    def display_loop(self):
        """图像显示循环"""
        cv2.namedWindow("黄灯检测与距离显示", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.image_lock:
                if self.latest_image is not None:
                    cv2.imshow("黄灯检测与距离显示", self.latest_image)

            # 等待按键事件，1ms超时
            key = cv2.waitKey(1)
            if key == 27:  # ESC键退出
                self.destroy_node()
                rclpy.shutdown()
                break
        cv2.destroyAllWindows()


class VoiceAnnouncer(Node):
    def __init__(self):
        super().__init__('voice_announcer_node')

        # 创建发布器，用于发送语音播报请求
        self.audio_pub = self.create_publisher(
            AudioPlayExtend,
            '/mi_desktop_48_b0_2d_7b_04_d3/speech_play_extend',
            10  # QoS服务质量设置
        )

        self.get_logger().info("Voice announcer node initialized and ready")

    def speak(self, text: str, module_name: str = "arrow_detector"):
        """发送语音播报请求"""
        if not rclpy.ok():
            self.get_logger().warn("ROS环境已关闭，无法发送语音消息")
            return

        msg = AudioPlayExtend()
        msg.is_online = True  # 使用在线TTS引擎
        msg.text = text       # 设置要播报的文本
        msg.module_name = module_name  # 设置模块名称

        self.audio_pub.publish(msg)
        self.get_logger().info(f"Published voice message: {text}")


def perform_countdown(announcer):
    """执行5秒倒计时并播报"""
    if not announcer or not rclpy.ok():
        return
        
    try:
        for i in range(5, 0, -1):
            announcer.speak(str(i))
            # 短暂等待，确保语音消息被处理
            time.sleep(0.9)  # 稍微短于1秒，补偿处理时间
    except Exception as e:
        announcer.get_logger().error(f"倒计时播报错误: {str(e)}")




def robot_control_thread(stop_event, detector, announcer):
    """机器人控制线程"""
    has_stopped = False
    Ctrl = EnhancedRobotCtrl()
    Ctrl.run()
    msg = robot_control_cmd_lcmt()
    
    try:
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)
        print("走")

        # 开始检测黄灯
        detector.set_detection_enabled(True)

        # 记录开始行走的时间
        start_walking_time = time.time()
        print(f"开始直行，记录起始时间 {start_walking_time}")
        
        # 开始直行
        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_SLOW
        msg.vel_des = [0.2, 0, 0]  # 向前直行
        msg.duration = 0
        msg.step_height = [0.06, 0.06]
        msg.life_count += 1
        Ctrl.Send_cmd(msg)

        # 持续直行，直到检测到黄灯或完成
        while rclpy.ok():
            # 检查是否需要停止，且之前未停止过
            if stop_event.is_set() and not has_stopped:
                # 再走一段距离
                msg.mode = 11
                msg.gait_id = 27
                msg.vel_des = [0.2, 0, 0]
                msg.duration = 0
                msg.step_height = [0.06, 0.06]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                print(f"检测到黄灯，距离小于{1.5}米，再走一段")
                time.sleep(5.5)

                # 停止移动
                msg.mode = 12  # 站立模式
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                # Ctrl.Wait_finish(12, 0)
                print("已停止")

                # 计算已行走时间
                if start_walking_time is not None:
                    walking_duration = time.time() - start_walking_time
                    print(f"检测到黄灯，停止前已行走：{walking_duration:.2f}秒")

                # 标记为已停止
                has_stopped = True

                time.sleep(0.5)  # 给语音服务一点准备时间
                perform_countdown(announcer)

                # 重置事件，继续直行
                stop_event.clear()
                msg.mode = 11  # 恢复运动
                msg.vel_des = [0.2, 0, 0]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                print("继续直行")
                
                # 确保不会出现负数睡眠
                continue_time = max(0, 45 - walking_duration)
                time.sleep(continue_time)

                # 最终停止
                msg.mode = 12  # 站立模式
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
                break

            # 每100ms检查一次
            time.sleep(0.1)

        #print('跳1')
        #msg.mode        = 16
        #msg.gait_id     = 3
        #msg.duration    = 1000
        #msg.life_count  += 1 
        #Ctrl.Send_cmd(msg)   
        #time.sleep(3.0)

    
        #print('跳2')
        #msg.mode        = 16
        #msg.gait_id     = 3
        #msg.duration    = 1000
        #msg.life_count  += 1
        #Ctrl.Send_cmd(msg)
        #time.sleep(3.0)   
        
        Ctrl.turn_to_direction(180)
        Ctrl.turn_to_direction(90)
        Ctrl.Wait_finish(11,27)

        msg.mode = 11 # Locomotion
        msg.gait_id = 27 # TROT_SLOW
        msg.vel_des = [-0.3, 0, 0 ]
        msg.duration = 0
        msg.step_height = [0.05, 0.05]
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(13) #持续时间

        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        print('跳1')
        msg.mode        = 16
        msg.gait_id     = 3
        msg.duration    = 1000
        msg.life_count  += 1 
        Ctrl.Send_cmd(msg)   
        time.sleep(3.0)

    
        print('跳2')
        msg.mode        = 16
        msg.gait_id     = 3
        msg.duration    = 1000
        msg.life_count  += 1
        Ctrl.Send_cmd(msg)
        time.sleep(3.0)   
    
        msg.mode        = 12
        msg.gait_id     = 0
        msg.life_count  += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        msg.mode = 11 # Locomotion
        msg.gait_id = 27
        msg.vel_des = [0.2, 0, 0 ]
        msg.duration = 0
        msg.step_height = [0.03, 0.03]
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(3) #持续时间


        print("正在下坡")
        msg.mode = 11
        msg.gait_id = 27
        msg.vel_des = [0.27, 0, 0]
        msg.step_height = [0.04, 0.06]
        msg.rpy_des = [0.0, 0.2, 0.0]
        msg.duration = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(10)

        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)


        

    except KeyboardInterrupt:
        msg.mode = 7  # PureDamper模式
        msg.gait_id = 0
        msg.duration = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        print(f"设置模式为 {msg.mode}，步态 ID 为 {msg.gait_id}")
    finally:
        sys.exit()


def main():
    # 初始化ROS
    rclpy.init(args=None)

    # 提前创建语音播报节点（关键优化点）
    announcer = VoiceAnnouncer()

    # 创建事件用于线程间通信
    stop_event = Event()

    # 创建黄灯检测器
    detector = YellowLightDetector(stop_event)

    # 使用多线程执行器，避免任务阻塞
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(detector)
    executor.add_node(announcer)

    # 启动机器人控制线程，传入提前初始化的announcer
    robot_thread = Thread(
        target=robot_control_thread, 
        args=(stop_event, detector, announcer),
        daemon=True)
    robot_thread.start()

    try:
        # 运行ROS循环
        executor.spin()
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        # 清理资源
        executor.remove_node(detector)
        executor.remove_node(announcer)
        detector.destroy_node()
        announcer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

class YellowLight_slope:
    def yellowLight_slope(self):
        # 初始化ROS
        rclpy.init(args=None)

        # 提前创建语音播报节点（关键优化点）
        announcer = VoiceAnnouncer()

        # 创建事件用于线程间通信
        stop_event = Event()

        # 创建黄灯检测器
        detector = YellowLightDetector(stop_event)

        # 使用多线程执行器，避免任务阻塞
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(detector)
        executor.add_node(announcer)

        # 启动机器人控制线程，传入提前初始化的announcer
        robot_thread = Thread(
            target=robot_control_thread, 
            args=(stop_event, detector, announcer),
            daemon=True)
        robot_thread.start()

        try:
            # 运行ROS循环
            executor.spin()
        except KeyboardInterrupt:
            print("用户中断程序")
        finally:
            # 清理资源
            executor.remove_node(detector)
            executor.remove_node(announcer)
            detector.destroy_node()
            announcer.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

