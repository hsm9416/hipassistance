#!/usr/bin/env python3
# -*- coding: utf-8 -*-  

import sys
import time
import pickle
import rospy
from std_msgs.msg import String,Int32
from uart import init_serial, uart_loop, toggle




class CollectImu:
    def __init__(self):
        rospy.Subscriber("/gait_phase", Int32, self.callback)
        self.current_value = 0

    def callback(self, data):
        self.current_value = data.data

    def display_robot_state(self,uart_data,current_value):
        if uart_data:
            print(f"gait_cycle: {self.current_value }, r_hip_angle: {uart_data[2]}")

    def gait_cycle(self):
        ser = init_serial()
        toggle()
        max_value = 99
        combined_logs = []
    
        for cycle in range(5):
            max_hip_angle_10_20_value = float('-inf')
            min_hip_angle_20_60_value = float('inf')
            max_hip_angle_60_80_value = float('-inf')
            max_hip_10_20_cv = 0
            min_hip_20_60_cv = 0
            max_hip_60_80_cv = 0
    
            # ROS의 메시지 콜백을 기다림
            rospy.wait_for_message("/gait_phase", Int32)
    
            # 각 current_value에 대해 UART 데이터 수집 및 처리
            while not rospy.is_shutdown():
                uart_data = uart_loop(ser)
    
                # 터미널에 현재 가이트 사이클과 힙 각도를 연속적으로 출력
                print(f"gait_cycle: {self.current_value}, r_hip_angle: {uart_data[2]}")
    
                # 최대 및 최소값 계산 및 해당 current_value 저장
                if 10 <= self.current_value <= 20:
                    if float(uart_data[2]) > max_hip_angle_10_20_value:
                        max_hip_angle_10_20_value = float(uart_data[2])
                        max_hip_10_20_cv = self.current_value 
                elif 20 < self.current_value <= 60:
                    if float(uart_data[2]) < min_hip_angle_20_60_value:
                        min_hip_angle_20_60_value = float(uart_data[2])
                        min_hip_20_60_cv = self.current_value 
                elif 60 < self.current_value <= 80:
                    if float(uart_data[2]) > max_hip_angle_60_80_value:
                        max_hip_angle_60_80_value = float(uart_data[2])
                        max_hip_60_80_cv = self.current_value 
    
                if self.current_value >= max_value:
                    break  # current_value가 max_value에 도달하면 사이클 종료
                
            # 각 사이클의 결과를 combined_logs에 저장
            combined_logs.append((max_hip_10_20_cv, max_hip_angle_10_20_value, 
                                  min_hip_20_60_cv, min_hip_angle_20_60_value, 
                                  max_hip_60_80_cv, max_hip_angle_60_80_value))

            if cycle ==5:
                print("gait finish!")
                break

        with open('combined_robot_state_logs.pkl', 'wb') as f:
            pickle.dump(combined_logs, f)

            print('save complete!')
    
        return combined_logs
    

def main():
    rospy.init_node('gait_phase_listener', anonymous=True)
    collect_imu = CollectImu()
    collect_imu.gait_cycle()
    rospy.spin()        


if __name__ == '__main__':
    try:
        # print("=========10========")
        # time.sleep(1)
        # print("=========9=========")
        # time.sleep(1)
        # print("=========8=========")
        # time.sleep(1)
        # print("=========7=========")
        # time.sleep(1)
        # print("=========6=========")
        # time.sleep(1)
        # print("=========5=========")
        # time.sleep(1)
        # print("=========4=========")
        # time.sleep(1)
        print("=========3=========")
        time.sleep(1)
        print("=========2=========")
        time.sleep(1)
        print("=========1=========")
        time.sleep(1)
    
        main()
    except rospy.ROSInterruptException:
        pass
