import cv2
import time
import numpy as np
from threading import Lock, Thread
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ehcd_rbctr import EnhancedRobotCtrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt


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

        # 消抖处理
        self.history = []
        self.stable_frames = 3

        # 新增：显示线程相关
        self.latest_display_image = None
        self.image_lock = Lock()
        self.display_thread = None

    def start(self):
        """启动 ROS 节点和图像订阅"""
        try:
            self.running = True
            # ROS2 接收线程
            self.ros_thread = Thread(target=self._ros_thread_func, daemon=True)
            self.ros_thread.start()

            # 独立显示线程
            self.display_thread = Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

            print("ROS2 箭头检测已启动")
            return True
        except Exception as e:
            print(f"无法启动 ROS2 订阅: {e}")
            return False

    def _display_loop(self):
        """独立线程负责刷新显示"""
        while self.running:
            with self.image_lock:
                if self.latest_display_image is not None:
                    cv2.imshow("Arrow Detection", self.latest_display_image)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC 退出
                self.stop()
                break

    def _ros_thread_func(self):
        """ROS2 线程函数"""
        rclpy.init()

        class ArrowDetectionNode(Node):
            def __init__(self, parent):
                super().__init__('arrow_detection_node')
                self.parent = parent
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,#BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=10
                )
                self.subscription = self.create_subscription(
                    Image,
                    'image_rgb',
                    self.image_callback,
                    qos
                )
                self.get_logger().info('箭头检测节点已启动,等待图像...')

            def image_callback(self, msg):
                try:
                    cv_image = self.parent.bridge.imgmsg_to_cv2(msg, 'bgr8')
                    direction = self.parent.detect_arrow_direction(cv_image)

                    if direction:
                        self.parent.history.append(direction)
                        if len(self.parent.history) > self.parent.stable_frames:
                            self.parent.history.pop(0)

                        # 连续相同方向，认为稳定
                        if len(self.parent.history) == self.parent.stable_frames and len(set(self.parent.history)) == 1:
                            with self.parent.detection_lock:
                                if direction != self.parent.last_detected_direction:
                                    self.parent.last_detected_direction = direction
                                    self.get_logger().info(f"检测到稳定箭头方向: {direction}")
                                    if self.parent.callback:
                                        self.parent.callback(direction)
                                    if self.parent.stop_after_detection:
                                        self.parent.stop()

                    # 准备显示的图像，不直接imshow
                    display_image = cv_image.copy()
                    if direction:
                        cv2.putText(display_image, f"Direction: {direction}",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 255, 0), 2)
                    with self.parent.image_lock:
                        self.parent.latest_display_image = display_image

                except Exception as e:
                    self.get_logger().error(f"处理图像时出错: {str(e)}")

        self.node = ArrowDetectionNode(self)
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def detect_arrow_direction(self, image):
        """绿色箭头方向检测 - 使用图像中点和质心相对位置"""
        try:
            # 获取图像尺寸
            height, width = image.shape[:2]
            center_x = width // 2  # 图像中心X坐标
            
            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            lower_green = np.array([35, 40, 40])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_info) == 3:
                _, contours, _ = contours_info
            else:
                contours, _ = contours_info

            if not contours:
                return None

            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) < 300:
                return None

            peri = cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, 0.03 * peri, True)
            if not (5 <= len(approx) <= 9):
                return None

            # 计算质心
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return None
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 判断方向：比较质心与图像中心的相对位置
            direction = None
            if cx > center_x + 2:  # 质心在中心右侧（阈值20像素）
                direction = "right"
            elif cx < center_x - 2:  # 质心在中心左侧
                direction = "left"
            else:
                return None  # 无法确定方向

            # 调试图像
            debug_img = image.copy()
            cv2.drawContours(debug_img, [largest_contour], -1, (0, 255, 0), 2)
            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)  # 质心红点
            cv2.line(debug_img, (center_x, 0), (center_x, height), (255, 0, 0), 2)  # 中心线
            
            # 添加方向指示
            if direction:
                cv2.putText(debug_img, f"Center: {center_x}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(debug_img, f"Centroid: {cx}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(debug_img, f"Direction: {direction}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            with self.image_lock:
                self.latest_display_image = debug_img

            return direction

        except Exception as e:
            print(f"箭头检测错误: {e}")
            return None


    def get_last_detected_direction(self):
        with self.detection_lock:
            return self.last_detected_direction

    def stop(self):
        self.running = False
        if self.ros_thread:
            self.ros_thread.join(timeout=2.0)
        if self.display_thread:
            self.display_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        print("ROS2箭头检测已停止")


class ArrowAction():
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

        self.detector = ArrowDetector(callback=self.arrow_callback, stop_after_detection=False)
        self.detector.start()

        try:
            while True:
                time.sleep(0.1)

            self.Ctrl.send_move_command(mode=11, gait_id=27, vel_des=[0.3, 0, 0], duration=1800,
                                        step_height=[0.06, 0.06])
            self.Ctrl.Wait_finish(11, 27)

            if self.direction == "right":
                self._execute_right_turn()
            elif self.direction == "left":
                self._execute_left_turn()

        except KeyboardInterrupt:
            print("正在退出...")
            self.Ctrl.send_move_command(12, 0, [0, 0, 0])
            self.Ctrl.Wait_finish(12, 0)
            self.cleanup()
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
