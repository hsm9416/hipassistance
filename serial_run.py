import serial
import time
import numpy as np
import math

import threading

last_l_hip_torque, last_r_hip_torque = -1001, -1001
# 시리얼 포트로부터 데이터를 읽어오는 스레드
class SerialReaderThread(threading.Thread):
    def __init__(self, ser):
        threading.Thread.__init__(self)
        self.ser = ser
        self.running = True

    def run(self):
        while self.running:
            #time.sleep(0.01)  # 1초 대기
            #print(self.ser.in_waiting)

            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8', errors="ignore")

                print("Data", data)
                
                if data:  # 데이터가 비어있지 않으면 처리
                    split_data = data.split('|')
                    if len(split_data) == 11:
                        (Robot_time, l_hip_angle, r_hip_angle,
                        l_hip_velocity, r_hip_velocity,
                        l_hip_torque, r_hip_torque,l_hip_current, r_hip_current,
                        control_mode, control_interval) = split_data

                        # 데이터 출력
                        # print(data)
                        print(f"Robot_time: {Robot_time}")
                        print(f"l_hip_angle: {l_hip_angle}")
                        print(f"r_hip_angle: {r_hip_angle}")
                        print(f"l_hip_velocity: {l_hip_velocity}")
                        print(f"r_hip_velocity: {r_hip_velocity}")
                        print(f"l_hip_current: {l_hip_current}")
                        print(f"r_hip_current: {r_hip_current}")
                        print(f"control_mode: {control_mode}")
                        print(f"control_interval: {control_interval}")
                        print("=====================================")
                        
    def stop(self):
        self.running = False


# 시리얼 객체 생성
ser = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)
      

def send_left_command(l_tau):
    l_command = ""
    if l_tau < 0:
        l_command = ("\r\n<2?0#-%04X>\r\n" % abs(l_tau)).encode()
    else:
        l_command = ("\r\n<2?0#+%04X>\r\n" % abs(l_tau)).encode()
    return l_command

def send_right_command(r_tau):
    r_command = ""
    if r_tau < 0:
        r_command = ("\r\n<2?1#-%04X>\r\n" % abs(r_tau)).encode()
    else:
        r_command = ("\r\n<2?1#+%04X>\r\n" % abs(r_tau)).encode()
    return r_command



def send_reset_command():
    l_command = f"\r\nCURRENT?L#-1001|\r\n".encode()
    r_command = f"\r\nCURRENT?R#-1001|\r\n".encode()
    return l_command,r_command

def main() : 
    try:
        # 사용자로부터 입력받기
        user_input = input("Enter '1' to send the command: ")
        # 시리얼 읽기 스레드 시작
        reader_thread = SerialReaderThread(ser)
        reader_thread.start()

        command = "\r\n<1?0#+0001>\r\n".encode()  # 문자열을 바이트로 인코딩
        ser.write(command)
        
        l_tau = 0 
        r_tau = 0

        totalCount = 0
        while True:   # 사용자가 '1'을 입력하면 명령 전송
            if user_input == '1':


                l_command = send_left_command(l_tau)
                r_command = send_right_command(r_tau)

                totalCount = totalCount + 1
                ser.write(l_command)
                time.sleep(0.002)
                totalCount = totalCount + 1
                ser.write(r_command)
                time.sleep(0.002)

                print(f"l_tau: {l_command}")
                print(f"r_tau: {r_command}")
                print(f"total_count: {totalCount}")
                print('===============================')
    

    except KeyboardInterrupt:

        #reader_thread.join()
                
        #send_left_command(-1001)
        #send_right_command(-1001)
        ser.flushOutput()
        reader_thread.stop()
        
        print("stopped")
        print("\nProgram terminated by user.")
        return

        

if __name__ == "__main__":
    main()
    time.sleep(2)
    command = "\r\n<1?0#+0000>\r\n".encode()  # 문자열을 바이트로 인코딩
    ser.write(command)

    time.sleep(1)

    
    ser.close()  # 시리얼 포트 닫기
    print("Serial port closed.")