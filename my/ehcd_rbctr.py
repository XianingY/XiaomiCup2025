import sys
import os
import time
from threading import Thread, Lock
import cv2
import numpy as np
import math as math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from robot_control import Robot_Ctrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from odo import OdomReceiver
from image_subscriber import ImageSubscriber


class EnhancedRobotCtrl(Robot_Ctrl):
    def __init__(self):
        # 调用父类的初始化方法
        super().__init__()

        self.response_lock = Lock()
        self.odo = OdomReceiver()
        self.current_life_count = 0

    def run(self):
        # 先调用父类的run方法
        super().run()

    def get_next_life_count(self):
        """获取下一个生命周期计数"""
        self.current_life_count += 1
        return self.current_life_count

    def send_move_command(self, mode, gait_id, vel_des, duration=0, step_height=None, rpy_des=None):
        """发送移动指令的统一方法"""
        msg = robot_control_cmd_lcmt()  # 内部创建消息对象
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


    def normalize_angle(self, angle):
        """将角度规范化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def degrees_to_radians(self, degrees):
        return degrees * math.pi / 180.0

    def turn_to_direction_old(self, direction, tolerance=0.03, max_wait=12,tag=0):
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

    def turn_to_direction(self, direction, tolerance=0.01, max_wait=12, max_attempts=3):
        """
        转向到指定方向，通过循环检测和调整直到角度误差小于容忍度

        Args:
            direction: 目标方向，可以是数字(度数)或方向字符串
            tolerance: 角度容差(弧度)
            max_wait: 单次转向最大等待时间(秒)
            max_attempts: 最大尝试次数
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

        # 转换为目标yaw角度(弧度)
        if target_degrees == 0:
            target_yaw = 0  # 前方
        elif target_degrees == 90:
            target_yaw = -math.pi / 2  # 右方
        elif target_degrees == 180:
            target_yaw = math.pi if now_yaw >= 0 else -math.pi  # 后方
        elif target_degrees == 270:
            target_yaw = math.pi / 2  # 左方
        else:
            target_yaw = -self.degrees_to_radians(target_degrees)
            target_yaw = self.normalize_angle(target_yaw)

        print(f"目标方向: {direction} -> {target_yaw:.3f} 弧度")

        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            print(f"\n=== 转向尝试 {attempt}/{max_attempts} ===")

            # 1. 执行转向
            if not self.turn_to_yaw(target_yaw, tolerance, max_wait):
                print("转向未完成，将继续尝试")
                continue

            # 2. 转向完成后检测当前角度
            current_yaw = self.odo.get_yaw()
            angle_error = abs(self.normalize_angle(current_yaw - target_yaw))
            print(f"转向后检测 - 当前角度: {current_yaw:.3f}, 误差: {angle_error:.3f}")

            # 3. 检查是否满足精度要求
            if angle_error <= tolerance:
                print(f"转向成功完成，最终角度误差: {angle_error:.3f} (<={tolerance})")
                return True

            # 4. 计算剩余需要转动的角度
            remaining_angle = self.normalize_angle(target_yaw - current_yaw)
            print(f"需要继续转动: {remaining_angle:.3f} 弧度")

            # 5. 如果剩余角度很小，直接微调
            if abs(remaining_angle) < 0.1:  # 约5.7度
                print("进行微调...")
                turn_msg = robot_control_cmd_lcmt()
                turn_msg.mode = 11
                turn_msg.gait_id = 26
                turn_msg.vel_des = [0, 0, 0.2 if remaining_angle > 0 else -0.2]
                turn_msg.duration = int(1000 * abs(remaining_angle))
                turn_msg.step_height = [0.06, 0.06]
                turn_msg.life_count = self.get_next_life_count()
                self.Send_cmd(turn_msg)
                time.sleep(abs(remaining_angle))  # 等待微调完成

        print(f"达到最大尝试次数 {max_attempts} 仍未达到精度要求")
        final_error = abs(self.normalize_angle(self.odo.get_yaw() - target_yaw))
        print(f"最终角度误差: {final_error:.3f}")
        return final_error <= tolerance * 2  # 放宽最终判断标准

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
        print(
            f"需要转动角度: {angle_diff:.3f} 弧度 ({angle_diff * 180 / math.pi:.1f}度) ({'左转' if angle_diff > 0 else '右转'})")

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
            return self.turn_to_direction(90)  # 右转90度
        else:
            return self.turn_to_direction(180)  # 180度转向

    def quit(self):

        # 调用父类的quit方法
        super().quit()
