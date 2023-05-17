import rclpy
from rclpy.node import Node

import os
import getpass
username = getpass.getuser()
import sys
sys.path.append("/home/"+username+"/catkin_ws/src/more_interfaces/msg")
import MotorVelocity
import EncoderValue
import MotorCommand

import time
import math
import serial

from threading import Lock

class MotorDriver(Node):
    def __init__(self):
        super().__init__('ROS2_serial_motor_driver')
        
        # declare parameter

        ## Encoder counts per revolution
        self.declare_parameter('encoder_cnt_per_revolution',value = 0)
        ## Execution rate of the stm32 F401RE code
        self.declare_parameter('loop_rate',value = 0)
        ## Serial port to connect
        self.declare_parameter('serial_port',value = "/dev/ttyUSB0")
        ## Serial baud rade
        self.declare_parameter('baud_rate',value = 115200)
        ## Enables debugging of serial commands
        self.declare_parameter('serial_debug',value = False)       
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        
        # check if not respond
        if(self.get_parameter('encoder_cnt_per_revolution').value == 0):
            print("Error : encoder_cnt_per_revolution is 0")
        if(self.get_parameter('loop_rate').value == 0):
            print("Error : loop_rate is 0")
        if(self.debug_serial_cmds):
            print("Serial debug enabled")

        
        self.subscription = self.create_subscription(
           
            # ROS2_serial_motor_command subscribes a Motor_command (which is rads/sec of the two motors)

            MotorCommand,
            'ROS2_seiral_motor_command',
            self.ROS2_serial_motor_command_callback,
            10
        
        )

        self.motor_speed_pub = self.create_publisher(MotorVelocity,'ROS2_serial_motor_vels',10)
        self.motor_encoder_pub = self.create_publisher(EncoderValue,'ROS2_serial_encoder_vals',10)



        ## setup memver variables
        self.last_encoder_timer = time.time()
        self.last_motor_left_encoder = 0
        self.last_motor_right_encoder = 0
        self.motor_left_speed = 0.0
        self.motor_right_speed = 0.0

        self.mutex = Lock()

        ## open serial

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")

    # made serial commands

    def send_pwm_motor_command(self,motor_left_pwm,motor_right_pwm):
        self.send_command(f"o {int(motor_left_pwm)} {int(motor_right_pwm)}")

    def send_feedback_motor_command(self,motor_left_encoder_cnt_per_loop,motor_right_encoder_cnt_per_loop):
        self.send_command(f"o {int(motor_left_encoder_cnt_per_loop)} {int(motor_right_encoder_cnt_per_loop)}")

    def send_encoder_read_command(self):
        respone = self.send_command(f"e")
        if respone:
            return [int(raw_encoder_cnt) for raw_encoder_cnt in respone.split()]
        return []
    
    def ROS2_serial_motor_command_callback(self, serial_command):
        if(serial_command.is_pwm):
            self.send_pwm_motor_command(serial_command.motor_left_required_rad_per_sec,serial_command.motor_right_required_rad_per_sec)
        else:
            scaler = (1/(2*math.pi)) * self.get_parameter('encoder_cnt_per_revolution').value * (1 / self.get_parameter('loop_rate').value)
            motor_left_cnt_per_loop = serial_command.motor_left_required_rad_per_sec * scaler
            motor_right_cnt_per_loop = serial_command.motor_right_required_rad_per_sec * scaler
            self.send_feedback_motor_command(motor_left_cnt_per_loop,motor_right_cnt_per_loop)

    def check_encoders(self):
        
        respone = self.send_encoder_read_command()
        
        if (respone):
            
            new_time = time.time()

            time_different = new_time - self.last_encoder_timer
            self.last_encoder_timer = new_time
            motor_left_dif = respone[0] - self.last_motor_left_encoder
            self.last_motor_left_encoder = respone[0]
            motor_right_dif = respone[1] - self.last_motor_right_encoder
            self.last_motor_right_encoder = respone[1]

            radian_per_cnt = 2*math.pi/self.get_parameter('encoder_cnt_per_revolution').value
            self.motor_left_speed = motor_left_dif*radian_per_cnt/time_different
            self.motor_right_speed = motor_right_dif*radian_per_cnt/time_different

            speed_msg = MotorVelocity()
            speed_msg.motor_left_rad_per_sec = self.motor_left_speed
            speed_msg.motor_right_rad_per_sec = self.motor_right_speed
            self.motor_speed_pub.publish(speed_msg)

            encoder_msg = EncoderValue()
            encoder_msg.motor_left_encoder_value = self.last_motor_left_encoder
            encoder_msg.motor_right_encoder_value = self.last_motor_right_encoder
            self.motor_encoder_pub.publish(encoder_msg)




    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()


def main(args=None):

    rclpy.init(args=args)

    serial_motor_driver = MotorDriver()

    rate = serial_motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(serial_motor_driver)
        serial_motor_driver.check_encoders()

    serial_motor_driver.close_conn()
    serial_motor_driver.destroy_node()
    rclpy.shutdown()