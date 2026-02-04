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

from green_arrow_direction import ArrowAction
from qrread import RobotQRRunner
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
    try:
        print("启动")
        a_runner = A_Scrue()
        print("A")
        slope_runner=Slope()
        print("S")
        b_stone_runner=B_Stone()
        print("BSTONE")

        aCode = a_runner.aArea()
        # print("test")
        a_runner.scrue()

        direction = slope_runner.arrow()
        
        if direction == "left":
            #石板路
            b_stone_runner.stonewalk()
            #限高杆
            b_stone_runner.downwalk()
            #B区
            a_runner.bArea(direction)
            #黄灯上下坡
            b_stone_runner.yellowLight_slope()
            #上下坡到S弯的衔接

        else :
            #上下坡黄灯
            b_stone_runner.slope_yellowLight()
            #B区
            a_runner.bArea(direction)
            #限高杆
            b_stone_runner.downwalk()
            #石板路
            b_stone_runner.stonewalk()
            #石板路到S弯的衔接

        a_runner.scrue_back(aCode)

        a_runner.back_to_station(aCode)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误：{e}")
    finally:
        sys.exit()



class A_Scrue:
    def aArea(self):
        print("aArea")
        # Ctrl = EnhancedRobotCtrl()
        # Ctrl.run()
        # odo = OdomReceiver()
                
        # Ctrl.quit()
        qr_runner = RobotQRRunner()
        aCode = qr_runner.qrReadA()
        return aCode

    def scrue(self):
        Ctrl = Robot_Ctrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()
        odo = OdomReceiver()
        
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
            msg.vel_des = [0.5, 0,-0.4]  # 转向 前 左 左转 速度
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
            Ctrl.quit()

    def scrue_back(self):
        print("S弯返回方法")

    
    def bArea(self,direction):
        print("bArea")
        qr_runner = RobotQRRunner()
        qr_runner.qrReadB(direction)

    #最后结尾处的行为
    def back_to_station(self,aCode):
        print("a_back")
        # Ctrl = EnhancedRobotCtrl()
        # Ctrl.run()
        # odo = OdomReceiver()
                
        # Ctrl.quit()
        qr_runner = RobotQRRunner()
        aCode = qr_runner.qrReadBack(aCode)
        return aCode
        

class Slope:
    def arrow(self):
        arrow_action = ArrowAction()
        direction = arrow_action.arrow_Read()
        return direction

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
    
    #黄灯
    def yellowLight_slope(self):
        print("黄灯_上下坡")
    def slope_yellowLight(self):
        print("上下坡_黄灯")
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

            Ctrl.send_move_command(7,0,[0, 0, 0])
            Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()



# Main function
if __name__ == '__main__':
    main()
