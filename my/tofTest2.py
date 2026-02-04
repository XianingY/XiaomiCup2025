'''
机器狗移动中的二维码识别代码
增强版MR813机器人控制，支持实时二维码识别功能
依赖库:
- robot_control_cmd_lcmt.py
- robot_control_response_lcmt.py
- opencv-python
- pyzbar (用于二维码识别)
- ROS2相关库 (rclpy, sensor_msgs, cv_bridge)
- pytesseract (用于OCR文字识别)
-
-调用示例：
-调用时引入RobotQRRunner
-from qrread import RobotQRRunner
-runner = RobotQRRunner()
-runner.qrReadA()
'''
import sys
import os
import time
from threading import Thread, Lock
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import math as math
from queue import Queue
import pytesseract
from PIL import Image as PILImage

# 导入Robot_Ctrl类
from robot_control import Robot_Ctrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from odo import OdomReceiver

# ROS2相关导入（在类外统一导入，便于Executor管理）
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from protocol.msg import AudioPlayExtend  # 导入CyberDog定义的音频消息类型

class QRCodeDetector(Node):
    def __init__(self, callback=None):
        super().__init__('qr_detection_node')
        self.callback = callback
        self.last_detected_data = None
        self.detection_lock = Lock()
        self.bridge = CvBridge()
        self.running = True

        self.window_name = 'QR Code Detection'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # 添加OCR相关配置
        self.ocr_config = r'--oem 3 --psm 6'  # OCR引擎配置
        self.target_texts = ['A-1', 'A-2', 'B-1', 'B-2']  # 要识别的目标文本

        # 设置QoS配置
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # 创建图像订阅者
        self.subscription = self.create_subscription(
            Image,
            '/image_rgb',  # 使用指定的话题
            self.image_callback,
            qos
        )
        self.get_logger().info('QR码检测节点已启动,等待图像...')

    def image_callback(self, msg):
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 读取二维码
            qr_data = self.read_qr_code(cv_image)

            if qr_data:
                with self.detection_lock:
                    if qr_data != self.last_detected_data:
                        self.last_detected_data = qr_data
                        self.get_logger().info(f"检测到新二维码: {qr_data}")

                        # 如果有回调函数，则调用它
                        if self.callback:
                            self.callback(qr_data)

            # 显示图像(可选)
            cv2.imshow('QR Code Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")

    def read_text(self, image):
        """使用OCR识别图像中的文本"""
        try:
            # 将OpenCV图像转换为PIL图像
            pil_image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            pil_image.info['dpi'] = (300, 300)  # 设置为300 dpi，提高识别精度

            # 使用Tesseract进行OCR识别
            text = pytesseract.image_to_string(pil_image, config=self.ocr_config)

            # 清理识别结果
            text = text.strip().replace(' ', '').replace('\n', '')

            # 检查是否识别到目标文本
            for target in self.target_texts:
                if target in text:
                    return target

            return None

        except Exception as e:
            print(f"文字识别过程中发生错误: {str(e)}")
            return None

    def read_qr_code(self, image):
        try:
            # 1. 原始图像直接识别
            decoded_objects = decode(image)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 2. 如果没有识别到二维码，尝试文字识别
            text_data = self.read_text(image)
            if text_data:
                print(f"识别到文字: {text_data}")
                return text_data

            # 3. 转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            decoded_objects = decode(gray)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 4. 自适应二值化处理
            adaptive_thresh = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2
            )
            decoded_objects = decode(adaptive_thresh)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 5. 应用高斯模糊减少噪声然后锐化
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            sharpened = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)
            decoded_objects = decode(sharpened)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 6. 增强对比度
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            decoded_objects = decode(enhanced)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 7. 边缘增强
            edges = cv2.Canny(gray, 100, 200)
            kernel = np.ones((5, 5), np.uint8)
            dilated = cv2.dilate(edges, kernel, iterations=1)
            decoded_objects = decode(dilated)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 8. 尝试不同的阈值
            for thresh_val in [100, 127, 150, 175]:
                _, binary = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY)
                decoded_objects = decode(binary)
                if decoded_objects:
                    qr_data = decoded_objects[0].data.decode('utf-8')
                    return qr_data

            # 没有识别到二维码
            return None

        except Exception as e:
            print(f"二维码识别过程中发生错误: {str(e)}")
            return None

    def get_last_detected_data(self):
        """获取最近一次检测到的二维码数据"""
        with self.detection_lock:
            return self.last_detected_data

    def stop(self):
        """停止二维码检测"""
        self.running = False
        cv2.destroyAllWindows()
        self.destroy_node()
        print("QR码检测节点已停止")

class ObstacleDetectionNode(Node):
    def __init__(self, parent):
        super().__init__('obstacle_detection_node')
        self.parent = parent

        # 设置QoS配置
        qos = QoSProfile(depth=10)

        # 创建障碍物检测订阅者
        self.subscription = self.create_subscription(
            String,
            '/mi_desktop_48_b0_2d_7b_04_d3/lying_down_obstacle_detection',
            self.obstacle_callback,
            qos
        )
        self.get_logger().info('障碍物检测节点已启动，等待数据...')

    def obstacle_callback(self, msg):
        try:
            data = msg.data.strip()
            with self.parent.left_count_lock:
                if data == "left":
                    self.parent.left_count += 1
                    if self.parent.left_count % 5 == 0:  # 每5次打印一次进度
                        self.get_logger().info(f"已连续收到{self.parent.left_count}次'left'")
                else:
                    # 如果收到非"left"数据，重置计数器
                    self.parent.left_count = 0
        except Exception as e:
            self.get_logger().error(f"处理障碍物数据时出错: {str(e)}")

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

class EnhancedRobotCtrl(Robot_Ctrl):
    """扩展Robot_Ctrl类，添加二维码响应功能"""
    def __init__(self):

    # 初始化rclpy（如果尚未初始化）
        if not rclpy.ok():
            rclpy.init(args=None)

        # 调用父类的初始化方法
        super().__init__()

        self.qr_queue = Queue()
        # 初始化二维码检测器
        self.qr_detector = QRCodeDetector(callback=self.on_qr_code_detected)
        self.is_qr_responding = False  # 标记是否正在响应二维码
        self.response_lock = Lock()
        self.odo = OdomReceiver()
        self.current_life_count = 0

        # 障碍物检测相关变量
        self.left_count = 0  # 连续收到"left"的计数
        self.left_count_lock = Lock()
        self.obstacle_node = ObstacleDetectionNode(self)
        self.obstacle_running = False

        self.announcer = VoiceAnnouncer()


        # 创建多线程执行器，管理多个ROS节点
        self.executor = MultiThreadedExecutor(num_threads=4)  # 4个线程处理并发任务
        self.executor.add_node(self.qr_detector)
        self.executor.add_node(self.obstacle_node)
        self.executor.add_node(self.announcer)

        # 启动执行器线程
        self.executor_thread = Thread(target=self._executor_thread_func, daemon=True)


    def _executor_thread_func(self):
        """执行器线程函数，处理ROS节点回调"""
        while rclpy.ok() and self.obstacle_running:
            self.executor.spin_once(timeout_sec=0.1)

    def run(self):
        # 先调用父类的run方法
        super().run()

        # 启动节点和执行器
        self.obstacle_running = True
        self.executor_thread.start()
        print("ROS2多线程执行器已启动")

    def wait_for_continuous_left(self, required_count=20, timeout=120):
        """等待连续收到指定次数的"left"数据"""
        print(f"等待连续{required_count}次收到'left'数据...")
        start_time = time.time()

        while True:
            with self.left_count_lock:
                current_count = self.left_count

            if current_count >= required_count:
                print(f"成功连续收到{required_count}次'left'数据")
                # 重置计数器，为下一次检测做准备
                with self.left_count_lock:
                    self.left_count = 0
                return True

            if time.time() - start_time > timeout:
                print(f"超时: 在{timeout}秒内未连续收到{required_count}次'left'数据")
                return False

            time.sleep(0.1)

    def get_next_life_count(self):
        """获取下一个生命周期计数"""
        self.current_life_count += 1
        return self.current_life_count

    def send_move_command(self, mode, gait_id, vel_des, duration, step_height=None, rpy_des=None):
        """发送移动指令的统一方法"""
        msg = robot_control_cmd_lcmt()
        msg.mode = mode
        msg.gait_id = gait_id
        msg.vel_des = vel_des
        msg.duration = duration
        if step_height:
            msg.step_height = step_height
        if rpy_des:
            msg.rpy_des = rpy_des
        msg.life_count = self.get_next_life_count()

        self.Send_cmd(msg)
        print(f"发送指令: mode={mode}, gait_id={gait_id}, vel_des={vel_des}, life_count={msg.life_count}")
        return msg

    def on_qr_code_detected(self, qr_data):
        """当检测到二维码时的回调函数"""
        with self.response_lock:
            if self.is_qr_responding:
                return  # 如果已经在响应中，则忽略新的二维码检测

            self.is_qr_responding = True

        try:
            print(f"检测到二维码: {qr_data}")
            self.qr_queue.put(qr_data)
            print(f"get{self.qr_queue.get()}")
            self.qr_queue.put(qr_data)
        except Exception as e:
            print(f"处理二维码时出错: {e}")

        finally:
            with self.response_lock:
                self.is_qr_responding = False

    # Direction==0 右转90
    # Direction==1 左转90
    # 2 掉头
    def normalize_angle(self, angle):
        """将角度规范化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def degrees_to_radians(self, degrees):
        """角度转弧度"""
        return degrees * math.pi / 180.0

    def turn_to_direction(self, direction, tolerance=0.03, max_wait=12, tag=0):
        """
        转向到指定方向

        Args:
            direction: 目标方向，可以是：
                    - 数字(度数): 0, 90, 180, 270, 360等
                    - 字符串: "前"(0°), "右"(90°), "后"(180°), "左"(270°)
            tolerance: 角度容差(弧度)，默认1度
            max_wait: 最大等待时间(秒)
        """
        # 将方向转换为目标yaw角度(弧度)
        if isinstance(direction, str):
            direction_map = {
                "前": 0, "右": -90, "后": 180, "左": 270,
                "north": 0, "east": 90, "south": 180, "west": 270,
                "n": 0, "e": 90, "s": 180, "w": 270
            }
            if direction in direction_map:
                target_degrees = direction_map[direction]
            else:
                raise ValueError(f"未知的方向字符串: {direction}")
        else:
            target_degrees = float(direction)

        # 将角度规范化到0-360范围
        target_degrees = target_degrees % 360
        now_yaw = self.odo.get_yaw()

        # 转换为弧度
        if target_degrees == 0:
            target_yaw = 0  # 前方
        elif target_degrees == 90:
            target_yaw = -math.pi/2  # 右方 (-1.5708)
        elif target_degrees == 180:
            if now_yaw < 0:
                target_yaw = -math.pi
            else:
                target_yaw = math.pi  # 后方 (3.1416 或 -3.1416)
        elif target_degrees == 270:
            target_yaw = math.pi/2  # 左方 (1.5708)
        else:
            target_yaw = -self.degrees_to_radians(target_degrees)
            target_yaw = self.normalize_angle(target_yaw)

        print(f"目标方向: {direction} -> {target_yaw:.3f} 弧度")

        current_yaw = self.odo.get_yaw()
        if self.turn_to_yaw(target_yaw, tolerance, max_wait) or abs(self.normalize_angle(self.odo.get_yaw() - target_yaw)) < tolerance:
            print(f"一次成功，{current_yaw:.3f} 弧度")
            return True
        elif tag == 1:
            return True
        elif abs(self.normalize_angle(self.odo.get_yaw() - target_yaw)) < 0.2:
            return True
        else:
            print(f"一次失败，{self.odo.get_yaw():.3f} 弧度")
            count = 0
            while not self.turn_to_yaw(target_yaw, tolerance, max_wait) and count < 2:
                if abs(self.normalize_angle(self.odo.get_yaw() - target_yaw)) < tolerance:
                    print(f"******重新转动之后,由于延迟最后记录的角度不符合,但是实际是符合的")
                    break
                count += 1
                print(f"角度不精确，当前角度,{self.odo.get_yaw():.3f}")

    def turn_to_yaw(self, target_yaw, tolerance=0.03, max_wait=12):
        """
        转向到指定的绝对yaw角度

        Args:
            target_yaw: 目标yaw角度(弧度)，绝对角度
            tolerance: 角度容差(弧度)
            max_wait: 最大等待时间(秒)
        """
        print(f"开始转向到目标角度: {target_yaw:.3f} 弧度")

        try:
            current_yaw = self.odo.get_yaw()
            print(f"当前yaw角度: {current_yaw:.3f} 弧度")
        except Exception as e:
            print(f"无法获取当前角度: {e}")
            return False

        # 计算最短路径的角度差
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        print(f"需要转动角度: {angle_diff:.3f} 弧度 ({angle_diff*180/math.pi:.1f}度) ({'左转' if angle_diff > 0 else '右转'})")

        # 如果角度差很小，直接返回
        if abs(angle_diff) < tolerance:
            print("已经在目标角度附近，无需转向")
            return True

        # 发送转向指令
        turn_msg = robot_control_cmd_lcmt()
        turn_msg.mode = 11
        turn_msg.gait_id = 26
        turn_msg.vel_des = [0, 0, 0.5 if angle_diff > 0 else -0.5]
        turn_msg.duration = int(2470 * abs(angle_diff))
        turn_msg.step_height = [0.06, 0.06]
        turn_msg.life_count = self.get_next_life_count()

        self.Send_cmd(turn_msg)
        print(f"转向指令已下发，duration: {turn_msg.duration}, life_count: {turn_msg.life_count}")

        # 轮询yaw，直到接近目标
        t_start = time.time()
        success = False
        while True:
            try:
                now_yaw = self.odo.get_yaw()
                # 计算当前角度与目标角度的最短距离
                current_err = abs(self.normalize_angle(now_yaw - target_yaw))
                print(f"当前yaw: {now_yaw:.3f}，距离目标: {current_err:.3f}")

                if current_err < tolerance + 0.005:
                    print("转向完成")
                    stop_msg = robot_control_cmd_lcmt()
                    stop_msg.mode = 11
                    stop_msg.gait_id = 26
                    stop_msg.vel_des = [0, 0, 0.0]  # 停止转动
                    stop_msg.duration = 0
                    stop_msg.life_count = self.get_next_life_count()
                    self.Send_cmd(stop_msg)
                    success = True
                    break
                if time.time() - t_start > max_wait:
                    print("转向超时，未到目标角度")
                    break
            except Exception as e:
                print(f"获取角度失败: {e}")
                if time.time() - t_start > max_wait:
                    break
            time.sleep(0.15)

        # 转向完成后，短暂停顿让机器人稳定
        time.sleep(0.5)
        print("转向流程结束")
        return success

    # 保持向后兼容的方法
    def turn(self, Direction, angle_rad=1.5707, tolerance=0.0174, max_wait=12):
        """
        保持向后兼容的转向方法

        Args:
            Direction: 转向方向，1=左转，0=右转，其他=180度转向
            angle_rad: 转向角度(弧度) - 仅用于兼容，实际使用固定角度
            tolerance: 角度容差(弧度)
            max_wait: 最大等待时间(秒)
        """
        if Direction == 1:
            return self.turn_to_direction(270)  # 左转90度
        elif Direction == 0:
            return self.turn_to_direction(90)   # 右转90度
        else:
            return self.turn_to_direction(180)  # 180度转向

    # def quit(self):
    #     # 停止执行器和节点
    #     self.obstacle_running = False
    #     if self.executor_thread and self.executor_thread.is_alive():
    #         self.executor_thread.join(timeout=2.0)

    #     # 销毁节点
    #     self.qr_detector.destroy_node()
    #     self.obstacle_node.destroy_node()

    #     # 关闭执行器
    #     self.executor.shutdown()

    #     # 调用父类的quit方法
    #     super().quit()
    #     print("EnhancedRobotCtrl已停止")

    def quit(self):
        # 停止执行器和节点
        self.obstacle_running = False
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)

        # 销毁ROS节点
        try:
            self.qr_detector.destroy_node()
        except Exception as e:
            print(f"销毁 qr_detector 出错: {e}")

        try:
            self.obstacle_node.destroy_node()
        except Exception as e:
            print(f"销毁 obstacle_node 出错: {e}")

        try:
            self.announcer.destroy_node()
        except Exception as e:
            print(f"销毁 announcer 出错: {e}")

        # 关闭执行器
        try:
            self.executor.shutdown()
        except Exception as e:
            print(f"执行器关闭异常: {e}")

        # 调用父类 quit
        super().quit()

        # 🔹 新增：彻底关闭 rclpy 上下文
        if rclpy.ok():
            try:
                rclpy.shutdown()
                print("rclpy 已正常关闭")
            except Exception as e:
                print(f"关闭 rclpy 出错: {e}")

        print("EnhancedRobotCtrl已完全停止")



class RobotQRRunner():
    def qrTest(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        try:
            print("=== 开始执行qrTest流程 ===")
            # 起立
            print("1. 机器人起立")
            Ctrl.send_move_command(12, 0, [0, 0, 0], duration=0)
            Ctrl.Wait_finish(12, 0)

            Ctrl.send_move_command(64, 0, [0, 0, 0], duration=1300)
            time.sleep(2)  # 原代码中200秒过长，改为2秒

        except KeyboardInterrupt:
            print("正在退出...")
            Ctrl.send_move_command(12, 0, [0, 0, 0], duration=0)
            Ctrl.Wait_finish(12, 0)
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
        finally:
            Ctrl.quit()

    # A处的行为
    def qrReadA(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        try:
            print("=== 开始执行qrReadA流程 ===")
            #起立
            print("1. 机器人起立")
            Ctrl.send_move_command(12, 0, [0, 0, 0],duration=0)
            Ctrl.Wait_finish(12, 0)
            time.sleep(1)

            #第一段直行
            print("2. 第一段直行")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=1400,step_height=[0.06, 0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(3)

            #第一次右转 - 使用新方法
            print("3. 第一次右转到90度")
            # Ctrl.turn_to_direction(90)  # 转到右方
            # Ctrl.turn_to_direction(90)
            Ctrl.send_move_command(11, 27, [0, 0, -0.5],duration=3700,step_height=[0.06, 0.06])
            print("第一次右转完成，准备继续")
            Ctrl.Wait_finish(11,27)
            time.sleep(4)

            print("4. 转弯后直行")
            Ctrl.send_move_command(11, 27, [0.2, 0, 0],duration=5000, step_height=[0.06, 0.06])  # 慢速靠近
            # Ctrl.Wait_finish(11, 27)
            time.sleep(5)

            if not Ctrl.qr_queue.empty():
                print("[提示] 已经提前识别到二维码，跳过保险动作")
            else:
                # 保险动作
                Ctrl.send_move_command(12, 0, [0, 0, 0], duration=0)
                time.sleep(3)

                Ctrl.send_move_command(62, 3, [0, 0, 0], duration=4000)
                time.sleep(5)

                Ctrl.send_move_command(12, 0, [0, 0, 0], duration=0)
                time.sleep(3)

            while True:
                qr_data = Ctrl.qr_queue.get()
                print(f"[主线程] 收到二维码: {qr_data}")

                if qr_data == "A-1":
                    self.do_action_a1(Ctrl)
                    break
                elif qr_data == "A-2":
                    self.do_action_a2(Ctrl)
                    break
                else:
                    print(f"未知二维码: {qr_data}")
                    break

        except KeyboardInterrupt:
            print("正在退出...")
            Ctrl.send_move_command(12, 0, [0, 0, 0])
            Ctrl.Wait_finish(12, 0)
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
        finally:
            Ctrl.quit()

        return  qr_data

    # B处的行为
    def qrReadB(self, direction):
        # 使用增强的机器人控制类
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            if direction == "right":
                # 首先让机器狗站立
                msg.mode = 12  # Recovery stand
                msg.gait_id = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)

                msg.mode = 11  # Locomotion
                msg.gait_id = 27  # TROT_SLOW
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.step_height = [0.06, 0.06]
                msg.duration = 3800
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
                Ctrl.Wait_finish(11, 27)

                msg.mode = 11  # Locomotion
                msg.gait_id = 27  # TROT_SLOW
                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 2000
                msg.step_height = [0.06, 0.06]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.mode = 62  # Position interpolation control
                msg.gait_id = 3
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(62, 3)
                time.sleep(0.5)

                msg.mode = 12  # Recovery stand
                msg.gait_id = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)
            else:
                # 首先让机器狗站立
                msg.mode = 12  # Recovery stand
                msg.gait_id = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)

                msg.mode = 11  # Locomotion
                msg.gait_id = 27  # TROT_SLOW
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.duration = 3800
                msg.step_height = [0.06, 0.06]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
                Ctrl.Wait_finish(11, 27)

                msg.mode = 11  # Locomotion
                msg.gait_id = 27  # TROT_SLOW
                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 2000
                msg.step_height = [0.06, 0.06]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.mode = 62  # Position interpolation control
                msg.gait_id = 3
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(62, 3)
                time.sleep(0.5)

                msg.mode = 12  # Recovery stand
                msg.gait_id = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish(12, 0)

            # 等待二维码识别
            while True:
                qr_data = Ctrl.qr_queue.get()
                print(f"[主线程] 收到二维码: {qr_data}")

                if qr_data == "B-1":
                    Ctrl.announcer.speak("B区库位1")
                    self.do_action_b1(Ctrl, direction)
                    break
                elif qr_data == "B-2":
                    Ctrl.announcer.speak("B区库位2")
                    self.do_action_b2(Ctrl, direction)
                    break
                else:
                    print(f"未知二维码: {qr_data}")
                    break

        except KeyboardInterrupt:
            # 退出前停止
            print("正在退出...")
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.vel_des = [0, 0, 0]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)
        finally:
            Ctrl.quit()
            sys.exit()

    # 最后结尾处的行为
    def qrReadBack(self,aCode):
        # 使用增强的机器人控制类
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            # 首先让机器狗站立
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_SLOW
            msg.vel_des = [0, 0, 0.5]  # 左转
            msg.duration = 4000
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 3800
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            #二维码是A-2，返回时在A1卸货
            if aCode == "A-2" :
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 1800
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.duration = 3650
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [-0.5, 0, 0]  # 倒车入库
                msg.duration = 1800
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.mode = 7  # 趴下5s
                msg.gait_id = 0
                msg.vel_des = [0, 0, 0]
                msg.duration = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)

                # 等待连续20次收到"left"数据
                if not Ctrl.wait_for_continuous_left(20):
                    print("未能检测到足够的'left'信号，继续执行但可能存在风险")

                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 2000
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 2000
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
            else :
                #A区二维码是A-2，卸货去A-1
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 1800
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.duration = 3650
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [-0.5, 0, 0]  # 倒车入库
                msg.duration = 1800
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.mode = 7  # 趴下5s
                msg.gait_id = 0
                msg.vel_des = [0, 0, 0]
                msg.duration = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)

                # 等待连续20次收到"left"数据
                if not Ctrl.wait_for_continuous_left(20):
                    print("未能检测到足够的'left'信号，继续执行但可能存在风险")

                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 2000
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0.5, 0, 0]  # 直行
                msg.duration = 2000
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)

                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.duration = 3600
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                time.sleep(0.5)
            
            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 3200
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0, 0, -0.5]  # 右转
            msg.duration = 3600
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [-0.5, 0, 0]  # 倒车入库
            msg.duration = 2000
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            # 保持程序运行，继续进行二维码检测
            while True:
                time.sleep(0.1)

        except KeyboardInterrupt:
            # 退出前停止
            print("正在退出...")
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.vel_des = [0, 0, 0]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

        Ctrl.quit()
        sys.exit()

    def do_action_a1(self, Ctrl):
        print("进入识别完成后续操作")
        # 先直行
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0.5, 0, 0],  # 直行
            duration=1600,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        print("再右转")
        # Ctrl.turn_to_direction(180)
        Ctrl.send_move_command(11, 27, [0, 0, -0.5],duration=3700,step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11, 27)
        time.sleep(1)

        # 恢复直行
        print("恢复直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前移动
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 左转（使用转向方法）
        print("再转")
        # Ctrl.turn_to_direction(90)  # 或使用90
        # Ctrl.Wait_finish(11, 27)
        Ctrl.send_move_command(11, 27, [0, 0, 0.5],duration=3700,step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11,27)
        time.sleep(1)

        # 倒车入库
        print("倒车入库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[-0.5, 0, 0],  # 向前移动
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 趴下
        print("趴下")
        Ctrl.send_move_command(
            mode=7,
            gait_id=1,
            vel_des=[0, 0, 0],
            duration=3000
        )
        Ctrl.Wait_finish(7, 1)

        # 等待连续20次收到"left"数据
        if not Ctrl.wait_for_continuous_left(20):
            print("未能检测到足够的'left'信号，继续执行但可能存在风险")

        # 站起来
        print("站起来")
        Ctrl.send_move_command(
            mode=12,
            gait_id=0,
            vel_des=[0, 0, 0],
            duration=0
        )
        Ctrl.Wait_finish(12, 0)

        # # 掉头（左转180度）
        # print("掉头")
        # Ctrl.turn_to_direction(180)
        # Ctrl.turn_to_direction(90)
        # Ctrl.Wait_finish(11, 27)

        # 出库
        print("出库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前移动
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(5)

        # 左转
        print("左转")
        # Ctrl.turn_to_direction(0)  # 或使用270
        # Ctrl.Wait_finish(11, 27)
        Ctrl.send_move_command(11, 27, [0, 0, 0.5],duration=3700,step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11,27)
        time.sleep(1)

        # 直行
        print("直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(4)

        # 左转
        print("左转")
        # Ctrl.turn_to_direction(270)
        # Ctrl.Wait_finish(11, 27)
        Ctrl.send_move_command(11, 27, [0, 0, 0.5],duration=3900,step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11,27)
        time.sleep(1)
        # 直行
        print("直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],
            duration=4000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 右转
        print("右转")
        # Ctrl.turn_to_direction(0)
        # Ctrl.Wait_finish(11, 27)
        Ctrl.send_move_command(11, 27, [0, 0, -0.5],duration=3700,step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11,27)
        time.sleep(1)

        # 直行
        print("直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],
            duration=1500,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

    def do_action_a2(self, Ctrl):
        print("进入识别完成后续操作")
        # 先直行
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0.5, 0, 0],  # 直行
            duration=1600,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        print("再左转")
        Ctrl.turn_to_direction(0)
        Ctrl.Wait_finish(11, 27)

        # 恢复直行
        print("恢复直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前移动
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 右转（使用转向方法）
        print("再转")
        Ctrl.turn_to_direction(90)  # 或使用90
        Ctrl.Wait_finish(11, 27)

        # 直行进库
        print("直行进库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前移动
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 趴下
        print("趴下")
        Ctrl.send_move_command(
            mode=7,
            gait_id=1,
            vel_des=[0, 0, 0],
            duration=3000
        )
        Ctrl.Wait_finish(7, 1)

        # 等待连续20次收到"left"数据
        if not Ctrl.wait_for_continuous_left(20):
            print("未能检测到足够的'left'信号，继续执行但可能存在风险")

        # 站起来
        print("站起来")
        Ctrl.send_move_command(
            mode=12,
            gait_id=0,
            vel_des=[0, 0, 0],
            duration=0
        )
        Ctrl.Wait_finish(12, 0)

        # # 掉头（左转180度）
        # print("掉头")
        # Ctrl.turn_to_direction(180)
        # Ctrl.turn_to_direction(90)
        # Ctrl.Wait_finish(11, 27)

        # 出库
        print("出库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前移动
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 右转
        print("右转")
        Ctrl.turn_to_direction(180)
        Ctrl.Wait_finish(11, 27)

        # 直行
        print("直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],
            duration=2000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 右转
        print("右转")
        Ctrl.turn_to_direction(270)
        Ctrl.Wait_finish(11, 27)

        # 直行
        print("直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],
            duration=4000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

        # 右转
        print("右转")
        Ctrl.turn_to_direction(0)
        Ctrl.Wait_finish(11, 27)
        
        # 直行
        print("直行")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],
            duration=4000,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(3)

    def do_action_b1(self, Ctrl, direction):
        print("[B-1] 进入识别完成后续操作")
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, 0.5],  # 左转
            duration=3800,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        time.sleep(0.5)
        # 直行
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0.5, 0, 0],  # 向前走
            duration=2100,
            step_height=[0.06, 0.06]
        )
        #Ctrl.Wait_finish(11, 27)
        time.sleep(2.1)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, 0.5],  # 左转
            duration=3800,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[-0.5, 0, 0],  # 倒车入库
            duration=1500,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        # 趴下
        print("[B-1] 趴下")
        Ctrl.send_move_command(
            mode=7,  # 坐下/趴下模式
            gait_id=1,
            vel_des=[0, 0, 0],
            duration=3000
        )
        Ctrl.Wait_finish(7, 1)

        # 等待连续20次收到"left"数据
        if not Ctrl.wait_for_continuous_left(20):
            print("未能检测到足够的'left'信号，继续执行但可能存在风险")

        print("[B-1] 站起来")
        Ctrl.send_move_command(
            mode=12,  # Recovery stand
            gait_id=0,
            vel_des=[0, 0, 0],
            duration=0
        )
        Ctrl.Wait_finish(12, 0)

        # 出库（可选）
        print("[B-1] 出库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前走
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0, 0, 0.5],  # 左转
            duration=4200,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 向前走
            duration=4000,
            step_height=[0.06, 0.06]
        )
        time.sleep(4.0)
        #Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, -0.5],  # 右转
            duration=3600,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[-0.5, 0, 0],  # 倒车入库
            duration=1800,
            step_height=[0.06, 0.06]
        )
        time.sleep(1.8)
        #Ctrl.Wait_finish(11, 27)

        # 趴下
        print("[B-1] 趴下")
        Ctrl.send_move_command(
            mode=7,  # 坐下/趴下模式
            gait_id=1,
            vel_des=[0, 0, 0],
            duration=3000
        )
        Ctrl.Wait_finish(7, 1)

        # 等待连续20次收到"left"数据
        if not Ctrl.wait_for_continuous_left(20):
            print("未能检测到足够的'left'信号，继续执行但可能存在风险")

        print("[B-1] 站起来")
        Ctrl.send_move_command(
            mode=12,  # Recovery stand
            gait_id=0,
            vel_des=[0, 0, 0],
            duration=0
        )
        Ctrl.Wait_finish(12, 0)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0.5, 0, 0],  # 出库
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        if direction == "right":
            Ctrl.turn_to_direction(180)
            Ctrl.Wait_finish(11, 27)

            Ctrl.send_move_command(
                mode=11,  # Locomotion
                gait_id=27,  # TROT_SLOW
                vel_des=[0.5, 0, 0],  # 直行
                duration=4000,
                step_height=[0.06, 0.06]
            )
            time.sleep(4.0)
            #Ctrl.Wait_finish(11, 27)

            Ctrl.send_move_command(
                mode=11,  # Locomotion
                gait_id=27,  # TROT_SLOW
                vel_des=[0, 0, 0.5],  # 左转
                duration=3600,
                step_height=[0.06, 0.06]
            )
            Ctrl.Wait_finish(11, 27)

    def do_action_b2(self, Ctrl, direction):
        print("[B-2] 进入识别完成后续操作")
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, -0.5],  # 右转走
            duration=3600,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        # 直行
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0.5, 0, 0],  # 向前走
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, -0.5],  # 右转
            duration=3600,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        # 倒车
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[-0.5, 0, 0],  # 倒车
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        # 趴下
        print("[B-2] 趴下")
        Ctrl.send_move_command(
            mode=7,  # 坐下/趴下模式
            gait_id=1,
            vel_des=[0, 0, 0],
            duration=3000
        )
        Ctrl.Wait_finish(7, 1)

        # 按需添加：可选择站起来退出库区
        print("[B-2] 站起来")
        Ctrl.send_move_command(
            mode=12,  # Recovery stand
            gait_id=0,
            vel_des=[0, 0, 0],
            duration=0
        )
        Ctrl.Wait_finish(12, 0)

        # 出库（可选）
        print("[B-2] 出库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 直行
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, -0.5],  # 右转
            duration=4000,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 直行
            duration=4000,
            step_height=[0.06, 0.06]
        )
        time.sleep(4.0)
        #Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[0, 0, 0.5],  # 左转
            duration=3600,
            step_height=[0.06, 0.06]
        )
        Ctrl.Wait_finish(11, 27)

        # 倒车
        Ctrl.send_move_command(
            mode=11,  # Locomotion
            gait_id=27,  # TROT_SLOW
            vel_des=[-0.5, 0, 0],  # 倒车
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        # 趴下
        print("[B-2] 趴下")
        Ctrl.send_move_command(
            mode=7,  # 坐下/趴下模式
            gait_id=1,
            vel_des=[0, 0, 0],
            duration=3000
        )
        Ctrl.Wait_finish(7, 1)

        # 按需添加：可选择站起来退出库区
        print("[B-2] 站起来")
        Ctrl.send_move_command(
            mode=12,  # Recovery stand
            gait_id=0,
            vel_des=[0, 0, 0],
            duration=0
        )
        Ctrl.Wait_finish(12, 0)

        # 出库（可选）
        print("[B-2] 出库")
        Ctrl.send_move_command(
            mode=11,
            gait_id=27,
            vel_des=[0.5, 0, 0],  # 直行
            duration=2000,
            step_height=[0.06, 0.06]
        )
        time.sleep(2.0)
        #Ctrl.Wait_finish(11, 27)

        if direction == "left":
            Ctrl.send_move_command(
                mode=11,  # Locomotion
                gait_id=27,  # TROT_SLOW
                vel_des=[0, 0, 0.5],  # 左转
                duration=3600,
                step_height=[0.06, 0.06]
            )
            Ctrl.Wait_finish(11, 27)

            Ctrl.send_move_command(
                mode=11,  # Locomotion
                gait_id=27,  # TROT_SLOW
                vel_des=[0.5, 0, 0],  # 直行
                duration=4000,
                step_height=[0.06, 0.06]
            )
            time.sleep(4.0)
            #Ctrl.Wait_finish(11, 27)

            Ctrl.send_move_command(
                mode=11,  # Locomotion
                gait_id=27,  # TROT_SLOW
                vel_des=[0, 0, -0.5],  # 右转
                duration=3600,
                step_height=[0.06, 0.06]
            )
            Ctrl.Wait_finish(11, 27)

def main():
    # 初始化rclpy
    if not rclpy.ok():
        rclpy.init(args=None)

    QRR = RobotQRRunner()
    # 可根据需要切换不同的测试模式
    # QRR.qrTest()
    QRR.qrReadB("left")
    #QRR.qrReadA()
    # QRR.qrReadBack()

    # 关闭rclpy
    if rclpy.ok():
        rclpy.shutdown()

# 程序入口
if __name__ == '__main__':
    main()

