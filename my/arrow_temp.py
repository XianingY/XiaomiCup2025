import cv2
import numpy as np
import time
from threading import Lock, Thread
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ehcd_rbctr import EnhancedRobotCtrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

class TemplateMatcher:
    """模板匹配核心类"""
    def __init__(self):
        # 加载预存模板（实际路径需根据您的文件位置修改）
        self.templates = {
            "left": self._process_template(cv2.imread("img/left.png", cv2.IMREAD_GRAYSCALE)),
            "right": self._process_template(cv2.imread("img/right.png", cv2.IMREAD_GRAYSCALE))
        }
        self.threshold = 0.75  # 匹配阈值
        self.scales = [0.9, 1.0, 1.1]  # 多尺度检测

    def _process_template(self, template):
        """模板预处理"""
        if template is None:
            raise ValueError("模板加载失败，请检查路径")
        return cv2.Canny(
            cv2.GaussianBlur(template, (3, 3), 0),
            50, 150
        )

    def match(self, frame):
        """执行多尺度模板匹配"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        processed = cv2.Canny(
            cv2.GaussianBlur(gray, (5, 5), 0),
            50, 150
        )
        
        best_match = None
        max_val = -1
        
        for direction, template in self.templates.items():
            for scale in self.scales:
                resized = cv2.resize(template, None, fx=scale, fy=scale)
                h, w = resized.shape
                
                # 确保模板不大于图像
                if h > processed.shape[0] or w > processed.shape[1]:
                    continue
                
                res = cv2.matchTemplate(processed, resized, cv2.TM_CCOEFF_NORMED)
                _, current_max, _, _ = cv2.minMaxLoc(res)
                
                if current_max > max_val and current_max > self.threshold:
                    max_val = current_max
                    best_match = direction
        
        return best_match

class ArrowDetector(object):
    def __init__(self, callback=None, stop_after_detection=False):
        self.callback = callback
        self.stop_after_detection = stop_after_detection
        self.last_detected_direction = None
        self.detection_lock = Lock()
        self.bridge = CvBridge()
        self.node = None
        self.running = False
        self.ros_thread = None
        self.matcher = TemplateMatcher()  # 替换为模板匹配器

        # 消抖处理
        self.history = []
        self.stable_frames = 3  

        # 显示相关
        self.latest_display_image = None
        self.image_lock = Lock()
        self.display_thread = None

    def start(self):
        """启动 ROS 节点和图像订阅"""
        try:
            self.running = True
            self.ros_thread = Thread(target=self._ros_thread_func, daemon=True)
            self.ros_thread.start()

            self.display_thread = Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

            print("ROS2 箭头检测已启动")
            return True
        except Exception as e:
            print(f"无法启动 ROS2 订阅: {e}")
            return False

    def _display_loop(self):
        """独立显示线程"""
        while self.running:
            with self.image_lock:
                if self.latest_display_image is not None:
                    cv2.imshow("Arrow Detection", self.latest_display_image)
            if cv2.waitKey(1) & 0xFF == 27:
                self.stop()

    def _ros_thread_func(self):
        """ROS2 线程函数"""
        rclpy.init()

        class ArrowDetectionNode(Node):
            def __init__(self, parent):
                super().__init__('arrow_detection_node')
                self.parent = parent
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=10
                )
                self.subscription = self.create_subscription(
                    Image,
                    'image_rgb',
                    self.image_callback,
                    qos
                )

            def image_callback(self, msg):
                try:
                    cv_image = self.parent.bridge.imgmsg_to_cv2(msg, 'bgr8')
                    direction = self.parent.matcher.match(cv_image)  # 使用模板匹配
                    
                    if direction:
                        self.parent._update_direction(direction, cv_image)
                    
                    # 更新显示
                    display_img = cv_image.copy()
                    if direction:
                        cv2.putText(display_img, f"Direction: {direction}",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.7, (0, 255, 0), 2)
                    with self.parent.image_lock:
                        self.parent.latest_display_image = display_img

                except Exception as e:
                    self.get_logger().error(f"处理图像时出错: {str(e)}")

        self.node = ArrowDetectionNode(self)
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def _update_direction(self, direction, frame):
        """更新方向检测结果"""
        self.history.append(direction)
        if len(self.history) > self.stable_frames:
            self.history.pop(0)

        # 连续稳定检测才确认
        if (len(self.history) == self.stable_frames and 
            len(set(self.history)) == 1):
            
            with self.detection_lock:
                if direction != self.last_detected_direction:
                    self.last_detected_direction = direction
                    print(f"确认箭头方向: {direction}")
                    
                    if self.callback:
                        self.callback(direction)
                    
                    if self.stop_after_detection:
                        self.stop()

    def stop(self):
        """停止检测"""
        self.running = False
        if self.ros_thread:
            self.ros_thread.join(timeout=2.0)
        if self.display_thread:
            self.display_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        print("ROS2箭头检测已停止")

class ArrowAction():
    """机器人控制类（完全保留原有实现）"""
    def __init__(self):
        self.direction = None
        self.detector = None
        self.Ctrl = EnhancedRobotCtrl()

    def arrow_callback(self, direction):
        print(f"回调收到箭头方向: {direction}")
        self.direction = direction

    def arrow_Read(self):
        self.Ctrl.run()
        self.Ctrl.send_move_command(mode=12, gait_id=0, vel_des=[0, 0, 0], duration=0)
        self.Ctrl.Wait_finish(12, 0)

        self.detector = ArrowDetector(callback=self.arrow_callback)
        self.detector.start()

        try:
            while True:
                time.sleep(0.1)
                
                if self.direction == "right":
                    self._execute_right_turn()
                    break
                elif self.direction == "left":
                    self._execute_left_turn()
                    break

        except KeyboardInterrupt:
            print("正在退出...")
        finally:
            self.cleanup()

    def _execute_right_turn(self):
        print("执行右转")
        self.Ctrl.turn_to_direction_old(0)
        self.Ctrl.Wait_finish(11, 27)
        self.Ctrl.send_move_command(mode=11, gait_id=27, vel_des=[0.5, 0, 0], duration=2200,
                                   step_height=[0.06, 0.06])
        self.Ctrl.Wait_finish(11, 27)
        self.Ctrl.turn_to_direction_old(270)
        self.Ctrl.Wait_finish(11, 27)

    def _execute_left_turn(self):
        print("执行左转")
        self.Ctrl.turn_to_direction_old(180)
        self.Ctrl.Wait_finish(11, 27)
        self.Ctrl.send_move_command(mode=11, gait_id=27, vel_des=[0.5, 0, 0], duration=2200,
                                   step_height=[0.06, 0.06])
        self.Ctrl.Wait_finish(11, 27)
        self.Ctrl.turn_to_direction_old(270)
        self.Ctrl.Wait_finish(11, 27)

    def cleanup(self):
        if self.detector:
            self.detector.stop()
        self.Ctrl.stop()

if __name__ == "__main__":
    arrow_action = ArrowAction()
    arrow_action.arrow_Read()
