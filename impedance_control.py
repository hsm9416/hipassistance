#!/usr/bin/env python3
# -*- coding: utf-8 -*-  

import rospy
import serial
import time
import threading
import queue
import numpy as np
from reference import process_sequences, load_robot_state, segment_data_into_sequences
from uart import init_serial, uart_loop
from test import send_left_command, send_right_command
from std_msgs.msg import String,Int32,Float64
# from impedance.msg import MyData
import os

ser = init_serial()

class ImpedanceControl:
    
    def __init__(self, mass, stiffness, damping):
        self.mass = mass
        self.stiffness = stiffness
        self.damping = damping
    
    def impedance_control(self, desired_angle, desired_velocity, angle, velocity):
        tau = self.stiffness * (desired_angle - angle) + self.damping * (desired_velocity - velocity)
        return tau

    def update_state(self, angle, velocity):
        self.angle = angle
        self.velocity = velocity


class DataReceiver(threading.Thread):
    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True
        self.processed_data = None

    def run(self):
        while self.running:
            self.processed_data = uart_loop(self.ser)
            if self.processed_data is None:
                print("No data or error in data reception.")
                continue
            # 여기서 데이터 처리 로직을 추가할 수 있습니다.

    def stop(self):
        self.running = False

class CommandSender(threading.Thread):
    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True
        self.command_queue = queue.Queue()

    def run(self):
        while self.running:
            try:
                l_command, r_command = self.command_queue.get(timeout=0.1)
                self.ser.write(l_command)
                self.ser.write(r_command)
            except queue.Empty:
                pass

    def send_command(self, l_command, r_command):
        self.command_queue.put((l_command, r_command))

    def stop(self):
        self.running = False

class CollectTopic:
    def __init__(self):
        rospy.Subscriber("/gait_phase", Int32, self.callback)
        self.current_value = 0

    def callback(self, data):
        self.current_value = data.data
        # print("Callback received data:", data.data)


collect_imu = CollectTopic()

def main():
    if not ser.is_open:
        ser.open()
    rospy.init_node('impedance_control_node', anonymous=True)

    robot_state_data = load_robot_state('combined_robot_state_logs.pkl')
    sequences = segment_data_into_sequences(robot_state_data)
    ref_point = process_sequences(sequences)
    max_hip_10_20_cv, min_hip_20_60_cv, max_hip_60_80_cv, max_hip_10_20_value, min_hip_20_60_value, max_hip_60_80_value = ref_point
  

    impedance_control_left = ImpedanceControl(0, 20, 20)
    impedance_control_right = ImpedanceControl(0, 20, 20)

    desired_velocity = 0  # 초기 설정 

    data_receiver = DataReceiver(ser)
    data_receiver.start()


    command_sender = CommandSender(ser)
    command_sender.start()



    try:
        while True: 
            if data_receiver.processed_data is not None:
                # 데이터 처리 및 임피던스 제어 계산

                current_gait_phase = collect_imu.current_value
                r_current_gait_phase = current_gait_phase + 50
                if r_current_gait_phase > 99: 
                    r_current_gait_phase -= 100

                Robot_time, l_hip_angle, r_hip_angle, l_hip_velocity, r_hip_velocity, l_hip_torque, r_hip_torque, l_hip_target_torque, r_hip_target_torque, control_mode, control_interval = data_receiver.processed_data

                l_hip = round(l_hip_angle, 2)
                r_hip = round(r_hip_angle, 2)
                l_vel = round(l_hip_velocity, 2)
                r_vel = round(r_hip_velocity, 2)

                l_desired_angle = 0 
                r_desired_angle = 0
                

                if max_hip_10_20_cv < current_gait_phase <= max_hip_10_20_cv : 
                    l_desired_angle = max_hip_10_20_value
                elif max_hip_10_20_cv < current_gait_phase <= 50:
                    l_desired_angle = min_hip_20_60_value
                elif 50 < current_gait_phase <= max_hip_60_80_cv :
                    l_desired_angle = max_hip_60_80_value



                if max_hip_10_20_cv < r_current_gait_phase <= max_hip_10_20_cv : 
                    r_desired_angle = max_hip_10_20_value
                elif max_hip_10_20_cv < r_current_gait_phase <= 50:
                    r_desired_angle = min_hip_20_60_value
                elif 50 < r_current_gait_phase <= max_hip_60_80_cv :
                    r_desired_angle = max_hip_60_80_value


                # r_desired_angle = -l_desired_angle

                l_tau = impedance_control_left.impedance_control(l_desired_angle, desired_velocity, l_hip, l_vel)
                r_tau = -impedance_control_right.impedance_control(r_desired_angle, desired_velocity, r_hip, r_vel)
                
                l_tau_limit_round = round(l_tau, 0)
                r_tau_limit_round = round(r_tau, 0)

                l_command = send_left_command(l_tau_limit_round)
                r_command = send_right_command(r_tau_limit_round)

                command_sender.send_command(l_command, r_command)

                impedance_control_left.update_state(l_hip, l_vel)
                impedance_control_right.update_state(r_hip, r_vel)
                

                l_hip_pub = rospy.Publisher('l_hip', Float64, queue_size=10)
                r_hip_pub = rospy.Publisher('r_hip', Float64, queue_size=10)
                l_desired_angle_pub = rospy.Publisher('l_desired_angle', Float64, queue_size=10)
                r_desired_angle_pub = rospy.Publisher('r_desired_angle', Float64, queue_size=10)
                r_gait_phase_pub = rospy.Publisher('r_current_gait_phase',Float64,queue_size=10)

                l_tau_pub = rospy.Publisher('l_tau', Float64, queue_size=10)
                r_tau_pub = rospy.Publisher('r_tau', Float64, queue_size=10)
                l_real_pub = rospy.Publisher('l_real', Float64, queue_size=10)
                r_real_pub = rospy.Publisher('r_real', Float64, queue_size=10)
 
                
                
                print('l_hip_torque', l_hip_torque)
                print('r_hip_torque', r_hip_torque)
                print('l_desired_angle', l_desired_angle)
                print('r_desired_angle', r_desired_angle)


                l_hip_pub.publish(Float64(l_hip))
                r_hip_pub.publish(Float64(r_hip))
                r_gait_phase_pub.publish(Float64(r_current_gait_phase))
                l_desired_angle_pub.publish(Float64(l_desired_angle))
                r_desired_angle_pub.publish(Float64(r_desired_angle))
                l_tau_pub.publish(Float64(l_tau_limit_round))
                r_tau_pub.publish(Float64(r_tau_limit_round))
                l_real_pub.publish(Float64(l_hip_torque))
                r_real_pub.publish(Float64(r_hip_torque))



            time.sleep(0.01)  # 작은 딜레이를 추가하여 CPU 사용률을 줄입니다.

    except KeyboardInterrupt:
        data_receiver.stop()
        command_sender.stop()
        data_receiver.join()
        command_sender.join()
        l_command = send_left_command(0)
        r_command = send_right_command(0)
        ser.write(l_command)
        ser.write(r_command)
        print("Program terminated by user.")

    finally:

        ser.close()
        print("Serial port closed.")
        

if __name__ == "__main__":
    main()
    rospy.spin() 
    
