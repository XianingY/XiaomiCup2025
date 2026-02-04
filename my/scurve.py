# -*- coding: utf-8 -*-
import lcm
import sys
import os
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from localization_lcmt import localization_lcmt

import toml
import copy
import math
import cv2
import numpy as np
from scipy.optimize import minimize
from file_send_lcmt import file_send_lcmt
from linecontrol import LineControl

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

class Scurve:
    def scurve(self):
        try:
            # 初始化控制器
            Ctrl = LineControl()
            Ctrl.run()
            msg = robot_control_cmd_lcmt()

            # 初始姿态调整 - 保持站立
            msg.mode = 12  # Recovery stand模式（站立）
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)
            print("机器人已进入站立状态，开始黄线居中调整（仅基于图像底部10%检测）")


            # 2. 第一次黄线检测与调整
            print("\n===== 第一次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)


            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, 0.8]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 5200
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            # time.sleep(7)  # 持续时间

            # 2. 第二次黄线检测与调整
            print("\n===== 第二次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)


            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(1.3)  # 持续时间
            Ctrl.Wait_finish(11,27)

            # 2. 第三次黄线检测与调整
            print("\n===== 第三次黄线检测与调整 =====")
            #Ctrl.perform_line_detection_and_adjustment(msg)

            # 第一段 5/4Π
            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, -0.78]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 6300
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(7)  # 持续时间

            # 2. 第四次黄线检测与调整
            print("\n===== 第四次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)


            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0.75]  # 转向 前 左 左转 速度
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            msg.duration = 5200  #5200
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)

            # 2. 第五次黄线检测与调整
            print("\n===== 第五次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)

            #msg.mode = 11
            #msg.gait_id = 27
            #msg.vel_des = [0,0, 0.5]
            #msg.duration = 0
            #msg.step_height = [0.06, 0.06]
            #msg.life_count += 1
            #Ctrl.Send_cmd(msg)
            #Ctrl.Wait_finish(11, 27)
            #time.sleep(0.5)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.3,0, 0]
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1.1)


            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0,-0.5]  # 转向 前 左 左转 速度
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            #msg.duration = 3120
            msg.duration = 0
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(2.4)

            # 2. 第六次黄线检测与调整
            print("\n===== 第六次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)


        except KeyboardInterrupt:
            print("\n用户中断程序")
        except Exception as e:
            print(f"发生错误: {str(e)}")
        finally:
            if 'Ctrl' in locals():
                Ctrl.quit()


    def scurve_return(self):
        try:
            # 初始化控制器
            Ctrl = LineControl()
            Ctrl.run()
            msg = robot_control_cmd_lcmt()

            #time.sleep(5)
            # 初始姿态调整 - 保持站立
            msg.mode = 12  # Recovery stand模式（站立）
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)
            print("机器人已进入站立状态，开始黄线居中调整（仅基于图像底部10%检测）")


            # 2. 第一次黄线检测与调整
            print("\n===== 第yi次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0.68]
            msg.duration = 0  # 保持相同时长
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(2.5)

                # 2. 第二次黄线检测与调整
            print("\n===== 第三次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, -0.75]
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1.5)

            # 2. 第三次黄线检测与调整
            print("\n===== 第四次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)

            # 第一段 5/4Π
            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, -0.7]  # 转向 前 左 左转 速度
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 0 
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(4)  # 持续时间

            print("\n===== 第五次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)



            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0, 0, -0.5]  # 转向 前 左 左转 速度
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 0
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1)  # 持续时间


            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.45, 0, 0]  # 转向 前 左 左转 速度
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 0
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1)  # 持续时间

                # 2. 第四次黄线检测与调整
            print("\n===== 第七次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)


            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0.88]
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1.8)


            # 2. 第五次黄线检测与调整
            print("\n===== 第八次黄线检测与调整 =====")
            #Ctrl.perform_line_detection_and_adjustment(msg)


            #msg.mode = 11
            #msg.gait_id = 27
            #msg.vel_des = [0, -0.1, 0]
            #msg.duration = 0
            #msg.step_height = [0.06, 0.06]
            #msg.life_count += 1
            #Ctrl.Send_cmd(msg)
            #Ctrl.Wait_finish(11, 27)
            #time.sleep(1.5)

            
            # 2. 第五次黄线检测与调整
            print("\n===== 第九次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)


            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0.75]
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(3.8)

             # 2. 第五次黄线检测与调整
            print("\n===== 第九次黄线检测与调整 =====")
            Ctrl.perform_line_detection_and_adjustment(msg)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0, 0, 0.5]
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1)
            
            
            
            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0]
            msg.duration = 1500
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(1.5)


            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, -0.78]
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            time.sleep(4.7)

            Ctrl.perform_line_detection_and_adjustment(msg)
            
            #msg.mode = 11
            #msg.gait_id = 27
            #msg.vel_des = [0.5, 0, 0]
            #msg.duration = 1500
            #msg.step_height = [0.06, 0.06]
            #msg.life_count += 1
            #Ctrl.Send_cmd(msg)
            #Ctrl.Wait_finish(11, 27)
            #time.sleep(1)

            msg.mode = 7
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)

            #Ctrl.perform_line_detection_and_adjustment(msg)
        except KeyboardInterrupt:
            print("\n用户中断程序")
        except Exception as e:
            print(f"发生错误: {str(e)}")
        finally:
            if 'Ctrl' in locals():
                Ctrl.quit()

def main():
    # 执行scurve动作
    scurve = Scurve()
    print("s弯动作")
    scurve.scurve()

    # 执行scurve_return动作
    print("s弯返回动作")
    #scurve.scurve_return()

if __name__ == '__main__':
    main()
