import sys
import os
import time
from threading import Thread, Lock
import cv2
import numpy as np
import lcm
import toml
import copy
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from robot_control import Robot_Ctrl
from ehcd_rbctr import EnhancedRobotCtrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from odo import OdomReceiver
from image_subscriber import ImageSubscriber
from image_subscriber1 import ImageSubscriber1
from file_send_lcmt import file_send_lcmt

robot_cmd = {
   'mode': 0, 'gait_id': 0, 'contact': 0, 'life_count': 0,
   'vel_des': [0.0, 0.0, 0.0],
    'rpy_des': [0.0, 0.0, 0.0],
    'pos_des': [0.0, 0.0, 0.0],
    'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point': [0.0, 0.0, 0.0],
    'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
   'step_height': [0.0, 0.0],
    'value': 0,  'duration': 0
    }

def main():
<<<<<<< HEAD
    a = Aclass()
    b = Bclass()
    c = Cclass()
    d = Dclass()
    # b.sToSlope()
    # c.slopewalk()
    # c.move()
    # c.downSlope()

    # d.detect()  # 黄灯站起来之后
    # d.bArea()
    # d.detect()

    # d.bOut() #出B
    # d.downwalk() #过限高杆
    # d.beforeStone() #石板路前位置调整
    # d.stonewalk()
    # d.stoneToScuve()

    # c.move()
=======
    try:
        a_runner = A_Scrue()
        slope_runner=Slope()
        b_stone_runner=B_Stone()

        a_runner.aArea()
        a_runner.scrue()
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19

        slope_runner.arrow()
        slope_runner.sToSlope()
        slope_runner.turnBeforeSlope()
        slope_runner.slopewalk()
        slope_runner.move()

<<<<<<< HEAD
    # try:
    a.aArea()

    # moveTo(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg,node)

    a.scurve()
    # sToSlope(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg,node)
    # slopewalk(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg)

    # bArea(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg, node)
    # downwalk(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg)
    # stonewalk(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg)

    # except KeyboardInterrupt:
    #     print("正在退出...")
    # finally:
    #     # node.set_detection_enable(False, False)
    #     # node.set_light_detection_enable(False)
    #     # node.destroy_node()
    #     # rclpy.shutdown()
    #     Ctrl.quit()
    #     sys.exit()


class Aclass:
    def aArea(self):
=======
        b_stone_runner.yellowlight_detect()    #黄灯站起来之后,靠左靠前启动比较好
        b_stone_runner.bArea1()
        b_stone_runner.bArea2()
        b_stone_runner.bOut()
        b_stone_runner.downwalk()
        b_stone_runner.stonewalk()
        b_stone_runner.stoneToScuve()

        a_runner.scrue_return()
        a_runner.back_to_station()
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit()



class A_Scrue:
    def aArea():
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        print("=== 开始执行qrReadA流程 ===")
        
        #起立
        print("1. 机器人起立")
        Ctrl.send_move_command(12, 0, [0, 0, 0],duration=0)
        Ctrl.Wait_finish(12, 0)
        time.sleep(1)

        #第一段直行
        print("2. 第一段直行")
        Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=1400,step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11, 27)
        # time.sleep(3)

        #第一次右转 - 使用新方法
        print("3. 第一次右转到90度")
        Ctrl.turn_to_direction_old(90)  # 转到右方
        print("第一次右转完成，准备继续")
        time.sleep(1)

        print("4. 转弯后直行")
        Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=3500, step_height=[0.06, 0.06])
        # Ctrl.Wait_finish(11, 27)
        time.sleep(6)
        # time.sleep(1)

        # print("5. 准备进入二维码识别区域")
        # Ctrl.send_move_command(11, 27, [0.5, 0, 0], step_height=[0.06, 0.06])
        # time.sleep(2)

        #左转到前方
        print("6. 左转到前方(0度)")
        Ctrl.turn_to_direction_old(0)  # 转到前方
        print("左转完成")

        # 恢复直行
        print("7. 左转后直行")
        Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=2000, step_height=[0.06, 0.06])
        # Ctrl.Wait_finish(11, 27)
        # time.sleep(3)
        time.sleep(5)

        # 再左转
        print("8. 再次左转到左方(270度)")
        Ctrl.turn_to_direction_old(270)  # 转到左方
        # Ctrl.turn_to_direction(270)
        print("再次左转完成")

        #直行进库
        print("9. 直行进库")
<<<<<<< HEAD
        Ctrl.send_move_command(11, 27, [0.5, 0, 0], duration=1700, step_height=[0.06, 0.06])
=======
        Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=2000, step_height=[0.06, 0.06])
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19
        # Ctrl.Wait_finish(11, 27)
        # time.sleep(3)
        time.sleep(5)

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
<<<<<<< HEAD
        Ctrl.turn_to_direction_old(180, tag=1)
        Ctrl.turn_to_direction_old(90)
=======
        Ctrl.turn_to_direction(180,tag=1)
        Ctrl.turn_to_direction(90)
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19
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
        Ctrl.turn_to_direction_old(180)
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
        Ctrl.turn_to_direction_old(270)
        Ctrl.turn_to_direction_old(270)
        # Ctrl.send_move_command(11, 27, [0, 0, -0.5], step_height=[0.06, 0.06])
        time.sleep(1)

        #进入二维码前直行道
        print("17. 进入二维码前直行道")
        Ctrl.send_move_command(11, 27, [0.5, 0, 0],duration=4050, step_height=[0.06, 0.06])
        # time.sleep(6.3)
        # Ctrl.Wait_finish(11, 27)
        time.sleep(7)

        #转到前方，准备进入s
        print("18. 转到前方，准备进入S区域")
<<<<<<< HEAD
        Ctrl.turn_to_direction_old(0)
        Ctrl.turn_to_direction_old(0)

=======
        Ctrl.turn_to_direction(0)
        Ctrl.turn_to_direction(0)
        
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19
        # #直行
        # print("19. 最后直行")
        # Ctrl.send_move_command(11, 27, [0.5, 0, 0], step_height=[0.06, 0.06])
        # time.sleep(2.5)

        # print("20. 最终站立") 
        # Ctrl.send_move_command(12, 0, [0, 0, 0])
        # Ctrl.Wait_finish(12, 0)

        print("21.调整")
        Ctrl.send_move_command(11, 27, [0, 0.2, 0],duration=500, step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11, 27)

        Ctrl.send_move_command(11, 27, [0.2, 0, 0],duration=2500, step_height=[0.06, 0.06])
        Ctrl.Wait_finish(11, 27)

        print("=== qrReadA流程执行完成 ===")
                
        Ctrl.quit()

    def scrue():
        Ctrl = Robot_Ctrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
        odo = OdomReceiver()
        rclpy.init()
        image_subscriber = ImageSubscriber1(Ctrl)

        image_subscriber.set_edge_detection_enable(True)
        # 调整检测敏感度
        image_subscriber.set_edge_detection_sensitivity(cooldown=1.5)
        
        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count += 1  # Command will take effect when life_count update
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, 0.9]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 5200
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            # time.sleep(7)  # 持续时间
            
        
            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)  # 持续时间
            
            # 第一段 5/4Π 
            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, -0.9]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
            msg.duration = 6300
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            # time.sleep(7.5)  # 持续时间

            msg.mode = 11 
            msg.gait_id = 27  
            msg.vel_des = [0.5, 0, 1.0]  # 转向 前 左 左转 速度
            msg.duration = 0 
            msg.step_height = [0.06, 0.06]  
            msg.life_count += 1
            msg.duration = 4700  #5200
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)

            msg.mode = 11 
            msg.gait_id = 27  
            msg.vel_des = [0.5, 0,-0.9]  # 转向 前 左 左转 速度
            msg.duration = 0 
            msg.step_height = [0.06, 0.06]  
            msg.life_count += 1
            msg.duration = 3120
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)
        finally:
            image_subscriber.destroy_node()
            rclpy.shutdown()
            Ctrl.quit()

    def scrue_return():
        """S弯返程函数 - 从出口返回到入口"""
        Ctrl = Robot_Ctrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
        odo = OdomReceiver()
        rclpy.init()
        image_subscriber = ImageSubscriber1(Ctrl)

        # 启用边缘检测（返程时同样需要）
        image_subscriber.set_edge_detection_enable(True)
        image_subscriber.set_edge_detection_sensitivity(cooldown=1.5)
        
        try:
            # 恢复站立姿态
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0.9]  
            msg.duration = 3120  # 保持相同时长
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)
            
            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, -1.0]  
            msg.duration = 4700
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0.9]  
            msg.duration = 6300
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, 0]  # 直行
            msg.duration = 0
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)  # 短暂直行

            msg.mode = 11
            msg.gait_id = 27
            msg.vel_des = [0.5, 0, -0.9]  
            msg.duration = 5200
            msg.step_height = [0.06, 0.06]
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11, 27)

            # # 最终停止并进入阻尼模式
            # msg.mode = 7  # PureDamper
            # msg.gait_id = 0
            # msg.life_count += 1
            # Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(7, 0)
            

            
        except Exception as e:
            print(f"S弯返程出错: {str(e)}")
            
        finally:
            image_subscriber.destroy_node()
            rclpy.shutdown()
            Ctrl.quit()


<<<<<<< HEAD
    def moveTo(Ctrl, msg, lcm_cmd, lcm_usergait, usergait_msg, node):
        msg.mode = 12  # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1  # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        # 开头调整
        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, 0.1]  # 转向 前 左 左转 速度
        msg.duration = 200  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(11, 27)

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0.2, 0]  # 转向 前 左 左转 速度
        msg.duration = 1300  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(11, 27)

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.5, 0, 0]  # 转向 前 左 左转 速度
        msg.duration = 2000  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(11, 27)
=======
    def scrue_return_with_visual_guidance():
        Ctrl = Robot_Ctrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
        odo = OdomReceiver()
        rclpy.init()
        image_subscriber = ImageSubscriber1(Ctrl)

        # 启用所有检测功能
        image_subscriber.set_edge_detection_enable(True)
        image_subscriber.set_edge_detection_sensitivity(cooldown=1.0)  # 返程时更敏感
        image_subscriber.set_detection_enable(True, allow_adjustment=True)  # 启用黄线跟踪
        
        try:
            msg.mode = 12
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            # 启动视觉检测线程
            import threading
            vision_thread = threading.Thread(target=lambda: rclpy.spin(image_subscriber))
            vision_thread.daemon = True
            vision_thread.start()

            # S弯返程路径（结合视觉引导）
            segments = [
                {"vel": [0.5, 0, 0.9], "duration": 3120, "desc": "右转段1"},
                {"vel": [0.5, 0, -1.0], "duration": 4700, "desc": "左转段2"},
                {"vel": [0.5, 0, 0.9], "duration": 6300, "desc": "右转段3"},
                {"vel": [0.5, 0, 0], "duration": 500, "desc": "直行过渡"},
                {"vel": [0.5, 0, -0.9], "duration": 5200, "desc": "左转回归"}
            ]
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19

            for i, segment in enumerate(segments, 1):        
                msg.mode = 11
                msg.gait_id = 27
                msg.vel_des = segment["vel"]
                msg.duration = segment["duration"]
                msg.step_height = [0.06, 0.06]
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                
                if segment["duration"] > 0:
                    Ctrl.Wait_finish(11, 27)
                else:
                    time.sleep(segment["duration"] / 1000.0)

<<<<<<< HEAD
    def scurve(self):
        Ctrl =EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
        # msg.mode = 12 # Recovery stand
        # msg.gait_id = 0
        # msg.life_count += 1 # Command will take effect when life_count update
        # Ctrl.Send_cmd(msg)
        # Ctrl.Wait_finish(12, 0)

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.25, 0, 0]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(1)  # 持续时间

        # msg.mode = 11 # Locomotion
        # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        # msg.vel_des = [0.5, 0, 0.92 ] #转向 前 左 左转 速度
        # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
        #                     # Continuous motion can interrupt non-zero duration interpolation motion
        # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # time.sleep(4 ) #持续时间

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.25, 0, 0.46]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(10)  # 持续时间

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.25, 0, 0.44]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(4)  # 持续时间
        # 总共6就差不多

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.25, 0, 0]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        # time.sleep(0.45) #持续时间
        time.sleep(1.6)  # 持续时间

        msg.mode = 11  # Locomotion
        msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.5, 0, -0.85]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(3)  # 持续时间
        # 6 is origin

        msg.mode = 12  # Locomotion
        msg.gait_id = 0  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26

        msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        while (1):
            time.sleep(0.1)

        # msg.mode = 11 # Locomotion
        # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        # msg.vel_des = [0.5, 0, -1.1 ] #转向 前 左 左转 速度
        # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
        #                     # Continuous motion can interrupt non-zero duration interpolation motion
        # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # # 1original
        # time.sleep(1.3) #持续时间

        # msg.mode = 11 # Locomotion
        # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        # msg.vel_des = [0.5, 0, 0 ] #转向 前 左 左转 速度
        # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
        #                     # Continuous motion can interrupt non-zero duration interpolation motion
        # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # time.sleep(0.5) #持续时间

        # msg.mode = 11 # Locomotion
        # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        # msg.vel_des = [0.5, 0, 0.9 ] #转向 前 左 左转 速度
        # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
        #                     # Continuous motion can interrupt non-zero duration interpolation motion
        # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # time.sleep(6) #持续时间

        # msg.mode = 11 # Locomotion
        # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        # msg.vel_des = [0.5, 0, -0.9 ] #转向 前 左 左转 速度
        # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
        #                     # Continuous motion can interrupt non-zero duration interpolation motion
        # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # time.sleep(3) #持续时间

        # #总共7.5就差不多
=======
            # 完成后停止
            # msg.mode = 7
            # msg.gait_id = 0
            # msg.life_count += 1
            # Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(7, 0)
            
        finally:
            image_subscriber.destroy_node()
            rclpy.shutdown()
            Ctrl.quit()
    

    #最后结尾处的行为
    def back_to_station(self):
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

            msg.vel_des = [0, 0, -0.5]  # 右转
            msg.duration = 3600
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)

            msg.vel_des = [-0.5, 0, 0]  # 后退
            msg.duration = 2000
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)
                
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
        
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19

class Slope:
    def arrow():
        try:
            Ctrl = EnhancedRobotCtrl()
            Ctrl.run()
            msg = robot_control_cmd_lcmt()

            rclpy.init()
            node = ImageSubscriber(Ctrl)
            # 启动ROS线程
            spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()

            print("开始箭头检测")
            node.set_arrow_detection_enable( True)  # 启用检测
            start_time = time.time()
            while time.time() - start_time < 10:# 最多等待10秒
                time.sleep(0.1)
            node.set_arrow_detection_enable( False)  # 禁用检测
            # node.destroy_node()
            # rclpy.shutdown()
            print("箭头检测完成")



        finally:
            Ctrl.quit()


    def sToSlope(self):
        try:
            Ctrl = EnhancedRobotCtrl()
            Ctrl.run()
            msg = robot_control_cmd_lcmt()
            
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()  # Command will take effect when life_count update
            print(msg)  # 打印步态信息
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            print("1准备直行")
            msg.mode = 11  
            msg.gait_id = 27
            msg.vel_des = [0.2, 0, 0]  # 直行 
            msg.duration = 1500
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)

            print("2准备转弯")
            Ctrl.turn_to_direction(0)  # 转到右方

            print("3准备直行")
            msg.mode = 11  
            msg.gait_id = 27
            msg.vel_des = [0.2, 0, 0]  # 直行 
            msg.duration = 5000
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)

            print("4准备转弯")
            Ctrl.turn_to_direction(270)  
        finally:
            Ctrl.quit()

    #上坡前位置微调
    def turnBeforeSlope(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()  # Command will take effect when life_count update
            print(msg)  # 打印步态信息
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            Ctrl.turn_to_direction(270)

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()


    def slopewalk(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        usergait_msg = file_send_lcmt()

        cmd_msg=msg

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()  # Command will take effect when life_count update
            print(msg)  # 打印步态信息
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            steps = toml.load("toml/Gait_Params_slopewalk.toml")
            full_steps = {'step': [robot_cmd]}
            k = 0
            for i in steps['step']:
                cmd = copy.deepcopy(robot_cmd)
                cmd['duration'] = i['duration']
                if i['type'] == 'usergait':
                    cmd['mode'] = 11  # LOCOMOTION
                    cmd['gait_id'] = 110  # USERGAIT
                    cmd['vel_des'] = i['body_vel_des']
                    cmd['rpy_des'] = i['body_pos_des'][0:3]
                    cmd['pos_des'] = i['body_pos_des'][3:6]
                    cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                    cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                    cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                    cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                    cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                    cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                    cmd['acc_des'] = i['weight']
                    cmd['value'] = i['use_mpc_traj']
                    cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                    cmd['ctrl_point'][2] = i['mu']
                if k == 0:
                    full_steps['step'] = [cmd]
                else:
                    full_steps['step'].append(cmd)
                k = k + 1
            f = open("toml/Gait_Params_slopewalk_full.toml", 'w')
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))
            f.close()

            file_obj_gait_def = open("toml/Gait_Def_slopewalk.toml", 'r')
            file_obj_gait_params = open("toml/Gait_Params_slopewalk_full.toml", 'r')
            usergait_msg.data = file_obj_gait_def.read()
            lcm_usergait.publish("user_gait_file", usergait_msg.encode())
            time.sleep(0.5)
            usergait_msg.data = file_obj_gait_params.read()
            lcm_usergait.publish("user_gait_file", usergait_msg.encode())
            print(usergait_msg.data)
            time.sleep(0.1)
            file_obj_gait_def.close()
            file_obj_gait_params.close()

            user_gait_list = open("toml/Usergait_List.toml", 'r')
            steps = toml.load(user_gait_list)
            for step in steps['step']:
                cmd_msg.mode = step['mode']
                cmd_msg.value = step['value']
                cmd_msg.contact = step['contact']
                cmd_msg.gait_id = step['gait_id']
                cmd_msg.duration = step['duration']
                cmd_msg.life_count =Ctrl.get_next_life_count()
                for i in range(3):
                    cmd_msg.vel_des[i] = step['vel_des'][i]
                    cmd_msg.rpy_des[i] = step['rpy_des'][i]
                    cmd_msg.pos_des[i] = step['pos_des'][i]
                    cmd_msg.acc_des[i] = step['acc_des'][i]
                    cmd_msg.acc_des[i + 3] = step['acc_des'][i + 3]
                    cmd_msg.foot_pose[i] = step['foot_pose'][i]
                    cmd_msg.ctrl_point[i] = step['ctrl_point'][i]
                for i in range(2):
                    cmd_msg.step_height[i] = step['step_height'][i]
                print(f"设置步态：模式为 {cmd_msg.mode}，步态 ID 为 {cmd_msg.gait_id}，速度为 {cmd_msg.vel_des}，步高为 {cmd_msg.step_height}，生命计数为 {cmd_msg.life_count}")  # 打印步态信息
                lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
                time.sleep(0.1)
            for i in range(175):  # 15s Heat beat It is used to maintain the heartbeat when life count is not updated
                lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
                print(f"wait{i}")
                time.sleep(0.2)  

        finally:
            Ctrl.quit()

    
    def move(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
    
        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()  # Command will take effect when life_count update
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11  # Locomotion
            msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.1, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.00]  # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            print(msg.gait_id)
            time.sleep(35)  # 持续时间  

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()


class B_Stone:
    def yellowlight_detect(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            # msg.mode = 11 # Locomotion
            # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            # msg.vel_des = [0, 0, -0.5 ] #转向 前 左 左转 速度
            # msg.duration = 3000 # Zero duration means continuous motion until a new command is used.
            #                 # Continuous motion can interrupt non-zero duration interpolation motion
            # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            # msg.life_count =Ctrl.get_next_life_count()
            # Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(11,27)

            Ctrl.turn_to_direction(270)

            rclpy.init()
            node = ImageSubscriber(Ctrl)
            # 启动ROS线程
            spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()

            print("开始颜色检测")
            node.set_detection_enable(True, True)  # 启用检测
            start_time = time.time()
            while time.time() - start_time < 10:# 最多等待10秒
                time.sleep(0.1)
            node.set_detection_enable(False, False)  # 禁用检测
            # node.destroy_node()
            # rclpy.shutdown()
            print("颜色检测完成1111")
        finally:
            Ctrl.quit()

<<<<<<< HEAD
    #石板路前位置调整
    def beforeStone(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            # msg.mode = 11  # Locomotion
            # msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            # msg.vel_des = [0, 0, 0.5]  # 转向 前 左 左转 速度
            # msg.duration = 3000  # Zero duration means continuous motion until a new command is used.
            # # Continuous motion can interrupt non-zero duration interpolation motion
            # msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            # msg.life_count += 1
            # Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(11, 27)

            # Ctrl.turn_to_direction(270)

            rclpy.init()
            node = ImageSubscriber(Ctrl)
            # 启动ROS线程
            spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()

            print("开始颜色检测")
            node.set_detection_enable(True, True)  # 启用检测
            start_time = time.time()
            while time.time() - start_time < 6:  # 最多等待10秒
                time.sleep(0.1)
            node.set_detection_enable(False, False)  # 禁用检测
            node.destroy_node()
            rclpy.shutdown()
            print("颜色检测完成1111")
        finally:
            Ctrl.quit()

    def bArea(self):
=======

    def bArea1(self):
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
<<<<<<< HEAD
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)
            
            msg.mode = 11  # Locomotion
            msg.gait_id = 27  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            msg.life_count += 1
=======
            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 0 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19
            Ctrl.Send_cmd(msg)
            time.sleep(9.5)

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 0 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(2)
            #不知道为什么不能和上面合并改成12
            

            # msg.mode = 11 # Locomotion
            # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            # msg.vel_des = [0, 0, 0.5 ] #转向 前 左 左转 速度
            # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
            #                 # Continuous motion can interrupt non-zero duration interpolation motion
            # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            # msg.life_count =Ctrl.get_next_life_count()
            # Ctrl.Send_cmd(msg)
            # time.sleep(9 ) #持续时间

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0, 0, -0.5 ] #转向 前 左 左转 速度
            msg.duration = 7500 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(0.5)


            msg.mode = 7 # Locomotion
            msg.gait_id = 1 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0, 0, 0 ] #转向 前 左 左转 速度
            msg.duration = 5000 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7,1)

            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 2000 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(0.5 ) #持续时间

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0,0, -0.5 ] #转向 前 左 左转 速度
            msg.duration = 4100 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(0.5 ) #持续时间

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 4100 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(0.5 ) #持续时间

        finally:
            Ctrl.quit()
            

    def bArea2(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
        try:
            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0,0, -0.5 ] #转向 前 左 左转 速度
            msg.duration = 3600 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 1800 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)


            print("掉头")
            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0,0, -0.5 ] #转向 前 左 左转 速度
            msg.duration = 7500 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)

            msg.mode = 7 # Locomotion
            msg.gait_id = 1 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0, 0, 0 ] #转向 前 左 左转 速度
            msg.duration = 5000 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7,1)

            print("重新站立")
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

        finally:
            Ctrl.quit()


    def bOut(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            Ctrl.turn_to_direction(90)

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 3200 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)


            # rclpy.init()
            # node = ImageSubscriber(Ctrl)
            # # 启动ROS线程
            # spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
            # spin_thread.start()

            # print("开始颜色检测")
            # node.set_detection_enable(True, True)  # 启用检测
            # start_time = time.time()
            # while time.time() - start_time < 10:# 最多等待10秒
            #     time.sleep(0.1)
            # node.set_detection_enable(False, False)  # 禁用检测
            # node.destroy_node()
            # rclpy.shutdown()
            # print("颜色检测完成1111")


            # msg.mode = 11 # Locomotion
            # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            # msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            # msg.duration = 3500 # Zero duration means continuous motion until a new command is used.
            #                 # Continuous motion can interrupt non-zero duration interpolation motion
            # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            # msg.life_count =Ctrl.get_next_life_count()
            # Ctrl.Send_cmd(msg)
            # Ctrl.Wait_finish(11,27)
        finally:
            Ctrl.quit()

    def downwalk(self):
        lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        usergait_msg = file_send_lcmt()
        msg = robot_control_cmd_lcmt()
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()

        try:

            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.5,0, 0 ] #转向 前 左 左转 速度
            msg.duration = 3000 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)

            steps = toml.load("toml/Gait_Params_downwalk.toml")
            full_steps = {'step': [robot_cmd]}
            k = 0
            for i in steps['step']:
                cmd = copy.deepcopy(robot_cmd)
                cmd['duration'] = i['duration']
                if i['type'] == 'usergait':
                    cmd['mode'] = 11  # LOCOMOTION
                    cmd['gait_id'] = 110  # USERGAIT
                    cmd['vel_des'] = i['body_vel_des']
                    cmd['rpy_des'] = i['body_pos_des'][0:3]
                    cmd['pos_des'] = i['body_pos_des'][3:6]
                    cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                    cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                    cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                    cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                    cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                    cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                    cmd['acc_des'] = i['weight']
                    cmd['value'] = i['use_mpc_traj']
                    cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                    cmd['ctrl_point'][2] = i['mu']
                if k == 0:
                    full_steps['step'] = [cmd]
                else:
                    full_steps['step'].append(cmd)
                k = k + 1
            f = open("toml/Gait_Params_downwalk_full.toml", 'w')
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))
            f.close()

            file_obj_gait_def = open("toml/Gait_Def_downwalk.toml", 'r')
            file_obj_gait_params = open("toml/Gait_Params_downwalk_full.toml", 'r')
            usergait_msg.data = file_obj_gait_def.read()
            lcm_usergait.publish("user_gait_file", usergait_msg.encode())
            time.sleep(0.5)
            usergait_msg.data = file_obj_gait_params.read()
            lcm_usergait.publish("user_gait_file", usergait_msg.encode())
            print(usergait_msg.data)
            time.sleep(0.1)
            file_obj_gait_def.close()
            file_obj_gait_params.close()

            user_gait_list = open("toml/Usergait_List.toml", 'r')
            steps = toml.load(user_gait_list)
            for step in steps['step']:
                msg.mode = step['mode']
                msg.value = step['value']
                msg.contact = step['contact']
                msg.gait_id = step['gait_id']
                msg.duration = step['duration']
                msg.life_count =Ctrl.get_next_life_count()
                for i in range(3):
                    msg.vel_des[i] = step['vel_des'][i]
                    msg.rpy_des[i] = step['rpy_des'][i]
                    msg.pos_des[i] = step['pos_des'][i]
                    msg.acc_des[i] = step['acc_des'][i]
                    msg.acc_des[i + 3] = step['acc_des'][i + 3]
                    msg.foot_pose[i] = step['foot_pose'][i]
                    msg.ctrl_point[i] = step['ctrl_point'][i]
                for i in range(2):
                    msg.step_height[i] = step['step_height'][i]
                print(f"设置步态：模式为 {msg.mode}，步态 ID 为 {msg.gait_id}，速度为 {msg.vel_des}，步高为 {msg.step_height}，生命计数为 {msg.life_count}")  # 打印步态信息
                lcm_cmd.publish("robot_control_cmd", msg.encode())
                time.sleep(0.1)
            for i in range(60):  # 36s Heat beat It is used to maintain the heartbeat when life count is not updated
                lcm_cmd.publish("robot_control_cmd", msg.encode())
                print(f"wait{i}")
                time.sleep(0.2)

            msg.mode = 11  # Locomotion
            msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.2, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.03, 0.03]  # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(2)  # 持续时间
        finally:
            Ctrl.quit()

    def stonewalk(self):

        lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        usergait_msg = file_send_lcmt()
        msg = robot_control_cmd_lcmt()
        Ctrl =  EnhancedRobotCtrl()
        Ctrl.run()
        try:

            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11 # Locomotion
            msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0,-0.1, 0 ] #转向 前 左 左转 速度
            msg.duration = 1500 # Zero duration means continuous motion until a new command is used.
                            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.06] # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(11,27)

            steps = toml.load("toml/Gait_Params_stonewalk.toml")
            full_steps = {'step': [robot_cmd]}
            k = 0
            for i in steps['step']:
                cmd = copy.deepcopy(robot_cmd)
                cmd['duration'] = i['duration']
                if i['type'] == 'usergait':
                    cmd['mode'] = 11  # LOCOMOTION
                    cmd['gait_id'] = 110  # USERGAIT
                    cmd['vel_des'] = i['body_vel_des']
                    cmd['rpy_des'] = i['body_pos_des'][0:3]
                    cmd['pos_des'] = i['body_pos_des'][3:6]
                    cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                    cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                    cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                    cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                    cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                    cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                    cmd['acc_des'] = i['weight']
                    cmd['value'] = i['use_mpc_traj']
                    cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                    cmd['ctrl_point'][2] = i['mu']
                if k == 0:
                    full_steps['step'] = [cmd]
                else:
                    full_steps['step'].append(cmd)
                k = k + 1
            f = open("toml/Gait_Params_stonewalk_full.toml", 'w')
            f.write("# Gait Params\n")
            f.writelines(toml.dumps(full_steps))
            f.close()
            file_obj_gait_def = open("toml/Gait_Def_stonewalk.toml", 'r')
            file_obj_gait_params = open("toml/Gait_Params_stonewalk_full.toml", 'r')
            usergait_msg.data = file_obj_gait_def.read()
            lcm_usergait.publish("user_gait_file", usergait_msg.encode())
            time.sleep(0.5)
            usergait_msg.data = file_obj_gait_params.read()
            lcm_usergait.publish("user_gait_file", usergait_msg.encode())
            print(usergait_msg.data)
            time.sleep(0.1)
            file_obj_gait_def.close()
            file_obj_gait_params.close()
            user_gait_list = open("toml/Usergait_List.toml", 'r')
            steps = toml.load(user_gait_list)
            for step in steps['step']:
                msg.mode = step['mode']
                msg.value = step['value']
                msg.contact = step['contact']
                msg.gait_id = step['gait_id']
                msg.duration = step['duration']
                msg.life_count =Ctrl.get_next_life_count()
                for i in range(3):
                    msg.vel_des[i] = step['vel_des'][i]
                    msg.rpy_des[i] = step['rpy_des'][i]
                    msg.pos_des[i] = step['pos_des'][i]
                    msg.acc_des[i] = step['acc_des'][i]
                    msg.acc_des[i + 3] = step['acc_des'][i + 3]
                    msg.foot_pose[i] = step['foot_pose'][i]
                    msg.ctrl_point[i] = step['ctrl_point'][i]
                for i in range(2):
                    msg.step_height[i] = step['step_height'][i]
                print(f"设置步态：模式为 {msg.mode}，步态 ID 为 {msg.gait_id}，速度为 {msg.vel_des}，步高为 {msg.step_height}，生命计数为 {msg.life_count}")  # 打印步态信息
                lcm_cmd.publish("robot_control_cmd", msg.encode())
                time.sleep(0.1)
            for i in range(125):  # 36s Heat beat It is used to maintain the heartbeat when life count is not updated
                lcm_cmd.publish("robot_control_cmd", msg.encode())
                print(f"wait{i}")
                time.sleep(0.2)
            msg.mode = 11  # Locomotion
            msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.2, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.03, 0.03]  # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            time.sleep(1)  # 持续时间
        finally:
            msg=0
            lcm_cmd = 0
            lcm_usergait = 0
            usergait_msg = 0
            
            Ctrl.quit()

    def stoneToScuve(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            Ctrl.send_move_command(12,0,[0, 0, 0])
            Ctrl.Wait_finish(12, 0)

            Ctrl.turn_to_direction_old(0)
            time.sleep(2)

            Ctrl.send_move_command(11,27,[0.5, 0, 0],2000)
            # Ctrl.Wait_finish(11, 27)
            time.sleep(6)

            Ctrl.turn_to_direction_old(90)

            Ctrl.send_move_command(11,27,[0.5, 0, 0],800)
            # Ctrl.Wait_finish(11, 27)
            time.sleep(6)

            Ctrl.send_move_command(12,0,[0, 0, 0])
            Ctrl.Wait_finish(12, 0)
<<<<<<< HEAD
=======

            Ctrl.send_move_command(7,0,[0, 0, 0])
            Ctrl.Wait_finish(7, 0)
>>>>>>> db4c68026e9d36e225622fbce910b1256c5d6d19

        finally:
            Ctrl.quit()



# Main function
if __name__ == '__main__':
    main()