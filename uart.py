# read_data.py 파일
import serial
import time


# 시리얼 객체 생성
def uart_open():
    ser = serial.Serial(
        port="/dev/ttyTHS0",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )
    try:
            command = "\r\n<1?0#+0001>\r\n".encode()  # 문자열을 바이트로 인코딩
            ser.write(command)
            print("Command sent, waiting for response...")
            # 데이터 읽기
            while True:
                data = ser.readline().decode('utf-8', errors="ignore").strip()  # 데이터 읽고, 디코딩
                if data:  # 데이터가 비어있지 않으면 출력
                    split_data = data.split('|')
                    Robot_time,l_hip_angle,r_hip_angle,l_hip_velocity,r_hip_velocity,l_hip_torque,r_hip_torque,l_hip_target_torque,r_hip_target_torque,control_mode,control_interval=split_data

                    # print(f"Robot_time: {Robot_time}")
                    print(f"l_hip_angle: {l_hip_angle}")
                    print(f"r_hip_angle: {r_hip_angle}")
                    print(f"l_hip_velocity: {l_hip_velocity}")
                    print(f"l_hip_velocity: {l_hip_velocity}")
                    print(f"l_hip_torque: {l_hip_torque}")
                    print(f"r_hip_torque: {r_hip_torque}")
                    
                    print("=====================================")
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        ser.close()  # 시리얼 포트 닫기
        print("Serial port closed.")
    
if __name__ == '__main__':
    uart_open()


# 시리얼 객체 생성 및 데이터 읽기 함수
def read_serial_data(ser):
    # time.sleep(0.01)  # 1초 대기
    data = ser.readline().decode('utf-8', errors="ignore").strip()
    return data

# 데이터 처리 함수
def process_data(data):
    split_data = data.split('|')
    if len(split_data) == 11:  # 데이터가 올바르게 split되었는지 확인
        # 데이터 타입을 변환합니다. 예시로 l_hip_angle과 l_hip_velocity을 float으로 변환합니다.
        try:
            split_data[0] = split_data[0]# Robot_time을 float로 변환
            split_data[1] = float(split_data[1])  # l_hip_angle을 float로 변환
            split_data[2] = float(split_data[2])  # r_hip_angle을 float로 변환
            split_data[3] = float(split_data[3])  # l_hip_velocity을 float로 변환
            split_data[4] = float(split_data[4])  # r_hip_velocity을 float로 변환
            split_data[5] = float(split_data[5])  # l_hip_current를 float로 변환
            split_data[6] = float(split_data[6])  # r_hip_current를 float로 변환
            split_data[7] = float(split_data[7])  # l_hip_target_torque를 float로 변환
            split_data[8] = float(split_data[8])  # r_hip_target_torque float로 변환
            split_data[9] = float(split_data[9])  # control_mode를 float로 변환
            split_data[10] = float(split_data[10])  # control_interval를 float로 변환
        except ValueError as e:  # 변환 중 오류가 발생할 경우 처리
            raise ValueError(f"Error converting data to correct types: {e}")
        return split_data  # 변환된 리스트 반환
    else:
        raise ValueError("Received data does not match expected format")


# 시리얼 포트 초기화 및 오픈

def init_serial():
    return serial.Serial(
        port="/dev/ttyTHS0",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )
ser = init_serial()

def toggle():
    command = "\r\n<1?0#+0001>\r\n".encode()  # 문자열을 바이트로 인코딩
    ser.write(command)

# 메인 데이터 읽기 및 처리 루프
def uart_loop(ser):
    try: 
        toggle()
        while True:
            data = read_serial_data(ser)
            if data:  # 데이터가 비어있지 않으면
                # 데이터 처리
                return process_data(data)
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    while True:
        ser = init_serial()  # 시리얼 포트 초기화
        processed_data = uart_loop(ser)  # 데이터 처리 루프 실행
        # 변수 할당
        Robot_time, l_hip_angle, r_hip_angle, l_hip_velocity, r_hip_velocity, l_hip_torque, r_hip_torque, l_hip_target_torque, r_hip_target_torque, control_mode, control_interval = processed_data
        print(processed_data)
