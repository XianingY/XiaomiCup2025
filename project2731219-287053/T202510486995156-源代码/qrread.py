'''
机器狗移动中的二维码识别代码
增强版MR813机器人控制，支持实时二维码识别功能
依赖库:
- robot_control_cmd_lcmt.py
- robot_control_response_lcmt.py
- opencv-python
- pyzbar (用于二维码识别)
- ROS2相关库 (rclpy, sensor_msgs, cv_bridge)
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

# 导入Robot_Ctrl类
from robot_control import Robot_Ctrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from odo import OdomReceiver

class QRCodeDetector(object):
    def __init__(self, callback=None):
        # 使用ROS2相关库
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        
        self.callback = callback
        self.last_detected_data = None
        self.detection_lock = Lock()
        self.bridge = CvBridge()
        self.node = None
        self.running = False
        self.ros_thread = None
        
    def start(self):
        """启动ROS节点和图像订阅"""
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
            from sensor_msgs.msg import Image
            
            # 创建一个线程来运行ROS节点
            self.running = True
            self.ros_thread = Thread(target=self._ros_thread_func)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            print("ROS2 图像订阅已启动")
            return True
        except ImportError as e:
            print(f"无法启动ROS2订阅: {e}")
            print("请确保已安装ROS2和相关依赖")
            return False
    
    def _ros_thread_func(self):
        """ROS2线程函数"""
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        from sensor_msgs.msg import Image
        
        # 初始化ROS2
        rclpy.init()
        
        # 创建ROS2节点
        class QRDetectionNode(Node):
            def __init__(self, parent):
                super().__init__('qr_detection_node')
                self.parent = parent
                
                # 设置QoS配置
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=10
                )
                
                # 创建图像订阅者
                self.subscription = self.create_subscription(
                    Image,
                    '/rgb_camera/image_raw',  # 使用指定的话题
                    self.image_callback,
                    qos
                )
                self.get_logger().info('QR码检测节点已启动,等待图像...')
            
            def image_callback(self, msg):
                try:
                    # 转换图像格式
                    cv_image = self.parent.bridge.imgmsg_to_cv2(msg, 'bgr8')
                    
                    # 读取二维码
                    qr_data = self.parent.read_qr_code(cv_image)
                    
                    if qr_data:
                        with self.parent.detection_lock:
                            if qr_data != self.parent.last_detected_data:
                                self.parent.last_detected_data = qr_data
                                self.get_logger().info(f"检测到新二维码: {qr_data}")
                                
                                if self.parent.callback:
                                    self.parent.callback(qr_data)
                    
                    # 显示图像
                    # cv2.imshow('QR Code Detection', cv_image)
                    # cv2.waitKey(1)
                    
                except Exception as e:
                    self.get_logger().error(f"处理图像时出错: {str(e)}")
        
        # 创建并启动节点
        self.node = QRDetectionNode(self)
        
        # 运行ROS2直到停止
        while self.running:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # 清理ROS2资源
        self.node.destroy_node()
        rclpy.shutdown()
    
    def read_qr_code(self, image):
        try:
            # 1. 原始图像直接识别
            decoded_objects = decode(image)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 2. 转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            decoded_objects = decode(gray)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 3. 自适应二值化处理
            adaptive_thresh = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2
            )
            decoded_objects = decode(adaptive_thresh)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 4. 应用高斯模糊减少噪声然后锐化
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            sharpened = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)
            decoded_objects = decode(sharpened)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 5. 增强对比度
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            decoded_objects = decode(enhanced)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 6. 边缘增强
            edges = cv2.Canny(gray, 100, 200)
            kernel = np.ones((5, 5), np.uint8)
            dilated = cv2.dilate(edges, kernel, iterations=1)
            decoded_objects = decode(dilated)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                return qr_data

            # 7. 尝试不同的阈值
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
        if self.ros_thread:
            self.ros_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        print("ROS2图像订阅已停止")

class EnhancedRobotCtrl(Robot_Ctrl):
    """扩展Robot_Ctrl类，添加二维码响应功能"""
    def __init__(self):
        # 调用父类的初始化方法
        super().__init__()
        
        # 初始化二维码检测器
        self.qr_detector = QRCodeDetector(callback=self.on_qr_code_detected)
        self.is_qr_responding = False  # 标记是否正在响应二维码
        self.response_lock = Lock()
        self.odo = OdomReceiver()
        self.current_life_count = 0

    def run(self):
        # 先调用父类的run方法
        super().run()
        
        # 启动二维码检测
        self.qr_detector.start()
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
            
            # 创建一个新的命令消息
            msg = robot_control_cmd_lcmt()
            
            # 根据二维码内容执行不同的动作
            if qr_data == "A-1":
                print("先直行")

                msg.mode = 11  # Locomotion
                msg.gait_id = 27  # TROT_SLOW
                msg.vel_des = [0.5, 0, 0]  # 先直行
                msg.duration = 0
                msg.step_height = [0.06, 0.06]
                msg.life_count = self.cmd_msg.life_count + 1
                self.Send_cmd(msg)
                time.sleep(2)

                print("再右转")
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.life_count += 1
                self.Send_cmd(msg)
                time.sleep(6)  # 5秒
                
                # 恢复直行
                msg.vel_des = [0.5, 0, 0]  # 向前移动
                msg.life_count += 1
                time.sleep(3)
                self.Send_cmd(msg)

                # 再转
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.life_count += 1
                time.sleep(5.5)
                self.Send_cmd(msg)
                
                #直行进库
                msg.vel_des = [0.5, 0, 0]  # 向前移动
                msg.life_count += 1
                time.sleep(2)
                self.Send_cmd(msg)

                #掉头
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.life_count += 1
                time.sleep(13.5)
                self.Send_cmd(msg)

                #出库
                msg.vel_des = [0.5, 0, 0]  # 向前移动
                msg.life_count += 1
                time.sleep(2)
                self.Send_cmd(msg)

                #左转
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.life_count += 1
                time.sleep(5)
                self.Send_cmd(msg)

                #直行
                msg.vel_des = [0.5, 0, 0]  
                msg.life_count += 1
                time.sleep(3)
                self.Send_cmd(msg)

                #左转
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.life_count += 1
                time.sleep(5.5)
                self.Send_cmd(msg)

                #直行
                msg.vel_des = [0.5, 0, 0]  # 
                msg.life_count += 1
                time.sleep(6)
                self.Send_cmd(msg)

                #右转
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.life_count += 1
                time.sleep(5)
                self.Send_cmd(msg)
                
            elif qr_data == "A-2":
                print("先直行")

                msg.mode = 11  # Locomotion
                msg.gait_id = 27  # TROT_SLOW
                msg.vel_des = [0.5, 0, 0]  # 先直行
                msg.duration = 0
                msg.step_height = [0.06, 0.06]
                msg.life_count = self.cmd_msg.life_count + 1
                self.Send_cmd(msg)
                time.sleep(2)

                print("再左转")
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.life_count += 1
                self.Send_cmd(msg)
                time.sleep(5.5)  # 

                # 恢复直行
                msg.vel_des = [0.5, 0, 0]  # 向前移动
                msg.life_count += 1
                time.sleep(3)
                self.Send_cmd(msg)

                # 再转
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.life_count += 1
                time.sleep(6)
                self.Send_cmd(msg)

                #直行进库
                msg.vel_des = [0.5, 0, 0]  # 向前移动
                msg.life_count += 1
                time.sleep(3)
                self.Send_cmd(msg)

                #掉头
                msg.vel_des = [0, 0, 0.5]  # 左转
                msg.life_count += 1
                time.sleep(12)
                self.Send_cmd(msg)

                #出库
                msg.vel_des = [0.5, 0, 0]  # 向前移动
                msg.life_count += 1
                time.sleep(3)
                self.Send_cmd(msg)

                #右转
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.life_count += 1
                time.sleep(5.5)
                self.Send_cmd(msg)

                #直行
                msg.vel_des = [0.5, 0, 0]  # 右转
                msg.life_count += 1
                time.sleep(3)
                self.Send_cmd(msg)

                #右转
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.life_count += 1
                time.sleep(6)
                self.Send_cmd(msg)

                #直行
                msg.vel_des = [0.5, 0, 0]  # 右转
                msg.life_count += 1
                time.sleep(6)
                self.Send_cmd(msg)

                #右转
                msg.vel_des = [0, 0, -0.5]  # 右转
                msg.life_count += 1
                time.sleep(6)
                self.Send_cmd(msg)

                msg.mode = 12 
                msg.gait_id = 0
                msg.life_count += 1 
                self.Send_cmd(msg)


            else:
                print(f"未知二维码: {qr_data}, 不执行动作")
                
        except Exception as e:
            print(f"处理二维码时出错: {e}")
            
        finally:
            with self.response_lock:
                self.is_qr_responding = False
    
    #2 掉头
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

    def turn_to_direction(self, direction, tolerance=0.03, max_wait=12,tag=0):
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
                "前": 0, "右": 90, "后": 180, "左": 270,
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
        # 机器人坐标系：0°=前方, 90°=左方, -90°=右方, 180°/-180°=后方
        # 我们的输入：0°=前方, 90°=右方, 180°=后方, 270°=左方
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
            # 对于其他角度，进行转换
            target_yaw = -self.degrees_to_radians(target_degrees)
            target_yaw = self.normalize_angle(target_yaw)
        
        print(f"目标方向: {direction} -> {target_yaw:.3f} 弧度")

        current_yaw = self.odo.get_yaw()
        if  self.turn_to_yaw(target_yaw, tolerance, max_wait) or abs(self.normalize_angle(self.odo.get_yaw() - target_yaw))<tolerance:
            print(f"一次成功，{current_yaw:.3f} 弧度")
            return True
        elif tag == 1:
            return True
        elif abs(self.normalize_angle(self.odo.get_yaw() - target_yaw))<0.2:
            return True
        else:
            # current_yaw = self.odo.get_yaw()
            print(f"一次失败，{self.odo.get_yaw():.3f} 弧度")
            count=0
            while not self.turn_to_yaw(target_yaw, tolerance, max_wait) and count<2 :
                if abs(self.normalize_angle(self.odo.get_yaw() - target_yaw))<tolerance:
                    print(f"******重新转动之后,由于延迟最后记录的角度不符合,但是实际是符合的")
                    break
                count+=1
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
        # if target_yaw*current_yaw<0:


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
                
                if current_err < tolerance+0.005:
                    print("转向完成")
                    success = True
                    break
                if time.time() - t_start > max_wait:
                    print("转向超时，未到目标角度")
                    print(f"{time.time()},{t_start}")
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
    
    def point_adjust(self):
        print(f"开始调节")
        
        try:
            now_x, now_y = self.odo.get_position()
            print(f"当前位置: x={now_x:.3f}, y={now_y:.3f}")
            
            target_x = 5.563735008239746
            target_y = 0.7217057347297668
            print(f"目标位置: x={target_x:.3f}, y={target_y:.3f}")

            x_diff = target_x - now_x
            y_diff = target_y - now_y
            print(f"位置差值: dx={x_diff:.3f}, dy={y_diff:.3f}")

            # 设置移动速度 (m/s)
            move_speed = 0.5
            
            # 计算移动时间，并确保是正数和合理范围
            x_duration = abs(x_diff) / move_speed
            y_duration = abs(y_diff) / move_speed
            
            # 设置最小移动时间和最大移动时间（避免过短或过长的移动）
            min_duration = 0.1  # 最小100ms
            max_duration = 10.0  # 最大10秒
            
            x_duration = max(min_duration, min(x_duration, max_duration))
            y_duration = max(min_duration, min(y_duration, max_duration))
            
            print(f"计算移动时间: x_duration={x_duration:.2f}s, y_duration={y_duration:.2f}s")

            # X轴调节
            if abs(x_diff) > 0.05:  # 只有差值大于5cm时才移动
                if now_x < target_x:
                    print("向X正方向移动")
                    self.send_move_command(11, 27, [move_speed, 0, 0], int(x_duration * 1000))  # 转换为毫秒
                else:
                    print("向X负方向移动")
                    self.send_move_command(11, 27, [-move_speed, 0, 0], int(x_duration * 1000))
                
                self.Wait_finish(11, 27)  # 修正参数分隔符
                print("X轴调节完成")
            else:
                print("X轴已在目标范围内，无需调节")

            # Y轴调节
            if abs(y_diff) > 0.05:  # 只有差值大于5cm时才移动
                if now_y < target_y:
                    print("向Y正方向移动")
                    self.send_move_command(11, 27, [0, move_speed, 0], int(y_duration * 1000))
                else:
                    print("向Y负方向移动")
                    self.send_move_command(11, 27, [0, -move_speed, 0], int(y_duration * 1000))
                
                self.Wait_finish(11, 27)  # 修正参数分隔符
                print("Y轴调节完成")
            else:
                print("Y轴已在目标范围内，无需调节")
            
            # 验证最终位置
            final_x, final_y = self.odo.get_position()
            final_error = ((final_x - target_x)**2 + (final_y - target_y)**2)**0.5
            print(f"调节完成！最终位置: x={final_x:.3f}, y={final_y:.3f}")
            print(f"最终误差: {final_error:.3f}m")
            
            return final_error < 0.1  # 返回是否成功（误差小于10cm）
        
        except Exception as e:
            print(f"位置调节失败: {e}")
            return False

    def quit(self):
        # 停止二维码检测
        self.qr_detector.stop()
        # 调用父类的quit方法
        super().quit()

class RobotQRRunner():
    #A处的行为
    def qrReadA(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        try:
            print("=== 开始执行qrReadA流程 ===")
            
            x,y = Ctrl.odo.get_position()
            print(f"当前位置{x}{y}")
            #起立
            print("1. 机器人起立")
            Ctrl.send_move_command(12, 0, [0, 0, 0],duration=0)
            Ctrl.Wait_finish(12, 0)
            time.sleep(1)

            #第一段直行
            print("2. 第一段直行")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=1500,step_height=[0.06, 0.06])
            Ctrl.Wait_finish(11, 27)
            # time.sleep(3)

            x,y = Ctrl.odo.get_position()
            print(f"当前位置{x}{y}")

            #第一次右转 - 使用新方法
            print("3. 第一次右转到90度")
            Ctrl.turn_to_direction(90)  # 转到右方
            Ctrl.turn_to_direction(90)
            print("第一次右转完成，准备继续")
            time.sleep(1)

            print("4. 转弯后直行")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=3500, step_height=[0.06, 0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(6)
            # time.sleep(1)

            x,y = Ctrl.odo.get_position()
            print(f"当前位置{x}{y}")

            # print("5. 准备进入二维码识别区域")
            # Ctrl.send_move_command(11, 27, [0.5, 0, 0], step_height=[0.06, 0.06])
            # time.sleep(2)

            #左转到前方
            print("6. 左转到前方(0度)")
            Ctrl.turn_to_direction(0)  # 转到前方
            print("左转完成")

            # 恢复直行
            print("7. 左转后直行")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=2000, step_height=[0.06, 0.06])
            # Ctrl.Wait_finish(11, 27)
            # time.sleep(3)
            time.sleep(5)

            x,y = Ctrl.odo.get_position()
            print(f"当前位置{x}{y}")

            # 再左转
            print("8. 再次左转到左方(270度)")
            Ctrl.turn_to_direction(270)  # 转到左方
            # Ctrl.turn_to_direction(270)
            print("再次左转完成")

            #直行进库
            print("9. 直行进库")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=2000, step_height=[0.06, 0.06])
            # Ctrl.Wait_finish(11, 27)
            # time.sleep(3)
            time.sleep(5)

            x,y = Ctrl.odo.get_position()
            print(f"当前位置{x}{y}")

            #趴下
            print("10. 趴下")
            Ctrl.send_move_command(7, 1, [0, 0, 0],duration=5000)
            Ctrl.Wait_finish(7,1)
            # time.sleep(1)
            
            #起立
            print("11. 重新起立")
            Ctrl.send_move_command(12, 0, [0, 0, 0],duration=0)
            Ctrl.Wait_finish(12, 0)
            # time.sleep(1)

            #掉头到右方
            print("12. 掉头到右方(90度)")
            Ctrl.turn_to_direction(180,tag=1)
            Ctrl.turn_to_direction(90)
            # Ctrl.send_move_command(11, 27, [0, 0, 0.5], step_height=[0.06, 0.06])
            time.sleep(1)

            #出库
            print("13. 出库")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=2000, step_height=[0.06, 0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(5)
            # time.sleep(1)

            #转到后方
            print("14. 转到后方(180度)")
            Ctrl.turn_to_direction(180)
            # Ctrl.send_move_command(11, 27, [0, 0, -0.5], step_height=[0.06, 0.06])
            time.sleep(1)

            #恢复直行
            print("15. 恢复直行")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=2500, step_height=[0.06, 0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(6)
            # time.sleep(1)

            #转到左方
            print("16. 转到左方(270度)")
            Ctrl.turn_to_direction(270)
            Ctrl.turn_to_direction(270)
            # Ctrl.send_move_command(11, 27, [0, 0, -0.5], step_height=[0.06, 0.06])
            time.sleep(1)

            #进入二维码前直行道
            print("17. 进入二维码前直行道")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=4100, step_height=[0.06, 0.06])
            # time.sleep(6.3)
            # Ctrl.Wait_finish(11, 27)
            time.sleep(7)

            #转到前方，准备进入s
            print("18. 转到前方，准备进入S区域")
            Ctrl.turn_to_direction(0)
            Ctrl.turn_to_direction(0)
            
            #直行
            print("19. 最后直行")
            Ctrl.send_move_command(11, 27, [0.5, 0, 0], duration=1000 ,step_height=[0.06, 0.06])
            time.sleep(5)

            Ctrl.point_adjust()

            print("20. 最终站立") 
            Ctrl.send_move_command(12, 0, [0, 0, 0])
            Ctrl.Wait_finish(12, 0)

            print("=== qrReadA流程执行完成 ===")
            
        except KeyboardInterrupt:
            print("正在退出...")
            Ctrl.send_move_command(12, 0, [0, 0, 0])
            Ctrl.Wait_finish(12, 0)
        except Exception as e:
            print(f"执行过程中发生错误: {e}")
        finally:
            Ctrl.quit()

    #B处的行为
    def qrReadB(self):
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
            #time.sleep(1)  # 给点时间稳定

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
            
            msg.mode = 62 # Position interpolation control
            msg.gait_id = 3 
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(62, 3)
            time.sleep( 0.5 )

            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)
            
            # msg.mode = 21 # Position interpolation control
            # msg.gait_id = 0
            # msg.rpy_des = [0, 0.5, 0] # Head up
            # msg.duration = 2000 # Expected execution time, 0.5s 
            # msg.life_count += 1
            # Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(21, 0)
            # time.sleep( 0.5 )

            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_SLOW
            msg.vel_des = [0, 0, -0.5]  # 右转
            msg.duration = 3600
            msg.step_height = [0.06, 0.06]
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

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 2000
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
            #已进库

            msg.vel_des = [0, 0, -0.5]  # 掉头
            msg.duration = 3600*2
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 2000
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
            #已出库

            msg.vel_des = [0, 0, -0.5]  # 右转
            msg.duration = 4300
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 4100
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
            

            msg.vel_des = [0, 0, -0.5]  # 右转
            msg.duration = 3600
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 2200
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
            #已进库

            msg.vel_des = [0, 0, -0.5]  # 掉头
            msg.duration = 8000
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 2500
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

    #最后结尾处的行为
    def qrReadBack(self):
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
            #time.sleep(1)  # 给点时间稳定

            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_SLOW
            msg.vel_des = [0, 0, 0.5]  # 左转
            msg.duration = 3600
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
            
            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 3800
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
            
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

            msg.vel_des = [0, 0, -0.5]  # 右转
            msg.duration = 3650
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 1800
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.mode = 7  # 趴下5s
            msg.gait_id = 0  
            msg.vel_des = [0, 0, 0]  
            msg.duration = 5000
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_SLOW
            msg.step_height = [0.06, 0.06]
            msg.vel_des = [0, 0, -0.5]  # 掉头
            msg.duration = 7200
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
            msg.duration = 3200
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


def main():
    QRR = RobotQRRunner()
    QRR.qrReadA()



# 程序入口
if __name__ == '__main__':
    main()