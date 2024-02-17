import serial
import time
import numpy as np
import math
import threading
import queue
from uart import init_serial, uart_loop
from test import send_left_command, send_right_command

ser = init_serial()


class DataReceiver(threading.Thread):
    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True
        self.processed_data = None

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

    def run(self):
        while self.running:
            self.processed_data = uart_loop(self.ser)
            if self.processed_data is None:
                print("No data or error in data reception.")
                continue

def left_friction_compensation(l_hip_velocity):
    if l_hip_velocity > 0:
           l_hip_velocity = l_hip_velocity*100
           Fc = 13
           Fs = 7.6047
           Vs = 30
           a1 = 0
           
           x1 = Fc
           x2 = Fs-Fc
           x3 = Vs
           x4 = a1
           l_tau = 1*(x1 + x2*np.exp(-((l_hip_velocity/x3)*(l_hip_velocity/x3))) + x4*l_hip_velocity)

    elif l_hip_velocity < 0:
              l_hip_velocity = l_hip_velocity*100
              Fc = 13
              Fs = 7.0963
              Vs = 20
              a1 = 0
              
              x1 = Fc
              x2 = Fs-Fc
              x3 = Vs
              x4 = a1
              l_tau = 1*(-x1 -x2*np.exp(-((l_hip_velocity/x3)*(l_hip_velocity/x3))) + x4*l_hip_velocity)  
    return l_tau

def right_friction_compensation(r_hip_velocity):

    if r_hip_velocity > 0:
         r_hip_velocity = r_hip_velocity*100
         x1 = -13
         x2 = 0.76713
         x3 = 29.9821
         x4 = -0.0006669
         r_tau = 1*(x1 + x2*np.exp(-((r_hip_velocity/x3)*(r_hip_velocity/x3))) + x4*r_hip_velocity)
    
    elif r_hip_velocity < 0:
        
         r_hip_velocity = r_hip_velocity*100
         x1 = -13
         x2 = 1.9768
         x3 = 19.9567
         x4 = -0.0047751
         r_tau = 1*(-x1 -x2*np.exp(-((r_hip_velocity/x3)*(r_hip_velocity/x3))) + x4*r_hip_velocity)

    return r_tau

def main():
    if not ser.is_open:
        ser.open()

    data_receiver = DataReceiver(ser)
    data_receiver.start()

    command_sender = CommandSender(ser)
    command_sender.start()
    
    try:
        while True: 
            if data_receiver.processed_data is not None:
                Robot_time, l_hip_angle, r_hip_angle, l_hip_velocity, r_hip_velocity, l_hip_torque, r_hip_torque, l_hip_target_torque, r_hip_target_torque, control_mode, control_interval = data_receiver.processed_data

                l_tau = left_friction_compensation(l_hip_velocity)
                r_tau = right_friction_compensation(r_hip_velocity)

                l_command = send_left_command(l_tau)
                r_command = send_right_command(r_tau)

                command_sender.send_command(l_command, r_command)

                print(f"l_tau: {l_tau}")
                print(f"r_tau: {r_tau}")
                print('===============================')

