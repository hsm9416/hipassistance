
##############################################################
###### constant, sine wave를 이용한 토크 명령 전송 테스트 code #####
##############################################################


import serial
import time
import numpy as np
import math

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
        l_command = ("\r\n<2?0#-%04X>\r\n" % abs(int(l_tau))).encode()
    else:
        l_command = ("\r\n<2?0#+%04X>\r\n" % abs(int(l_tau))).encode()
    return l_command

def send_right_command(r_tau):
    r_command = ""
    if r_tau < 0:
        r_command = ("\r\n<2?1#-%04X>\r\n" % abs(int(r_tau))).encode()
    else:
        r_command = ("\r\n<2?1#+%04X>\r\n" % abs(int(r_tau))).encode()
    return r_command

def main() : 
    try:
        # 사용자로부터 입력받기
        user_input = input("Enter '1' to send the command: ")

        while True:   # 사용자가 '1'을 입력하면 명령 전송
            if user_input == '1':

                l_tau = -200 
                r_tau = 200
    
                l_command = send_left_command(l_tau)
                r_command = send_right_command(r_tau)


                ser.write(l_command)
                ser.write(r_command)


                time.sleep(0.01)

                print(f"l_tau: {l_command}")
                print(f"r_tau: {r_command}")
                print('===============================')
    

    except KeyboardInterrupt:
        l_zero_command = send_left_command(0)
        r_zero_command = send_right_command(0)
        ser.write(l_zero_command)
        ser.write(r_zero_command)


        print("stopped")
        print("\nProgram terminated by user.")
    finally:
        ser.close()  # 시리얼 포트 닫기
        print("Serial port closed.")

if __name__ == "__main__":
    main()
