import sys

from my_interfaces.msg import Encoder, MotorCommand, MotorVels

import rclpy 
from rclpy.node import Node
import time
from tkinter import *
import math

import os
import getpass
username = getpass.getuser()
# sys.path.append("/home/iljujjang/catkin_ws/src/my_interfaces/msg")



class MotorGui(Node):
    def __init__(self):
        super().__init__('serial_motor_gui')

        #publish gui motor command to ~ 
        self.publisher = self.create_publisher(MotorCommand, 'serial_motor_command',10)


        #subscription for motor velocity
        self.velocity_subscription = self.create_subscription(
            MotorVels,
            'motor_velocitys',
            self.motor_velocitys_callback,
            10
        )
        #subscription for encoder value
        self.encoder_subscription = self.create_subscription(
            Encoder,
            'encoder_values',
            self.encoder_values_callback,
            10
        )

        #design gui
        self.tk = Tk()
        self.tk.title("leeilju's serial")
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        Label(root, text = "serial motor control and feedback").pack()

        mode_frame = Frame(root)
        mode_frame.pack(fill = X)

        self.mode_lbl = Label(mode_frame, text="zzzz")
        self.mode_lbl.pack(side=LEFT)
        self.mode_btn = Button(mode_frame, text="zzzz", command=self.switch_mode)
        self.mode_btn.pack(expand=True)

        slider_max_frame = Frame(root)
        slider_max_frame.pack(fill=X)
        self.slider_max_label = Label(slider_max_frame, text="Max Rev/sec", state="disable")
        self.slider_max_label.pack(side=LEFT)
        self.slider_max_val_box = Entry(slider_max_frame, state="disabled")
        self.slider_max_val_box.pack(side=LEFT)
        self.max_val_update_btn = Button(slider_max_frame, text='Update', command=self.update_scale_limits, state="disabled")
        self.max_val_update_btn.pack(side=LEFT)


        motor_left_frame = Frame(root)
        motor_left_frame.pack(fill=X)
        Label(motor_left_frame, text="Motor Left").pack(side=LEFT)
        self.motor_left = Scale(motor_left_frame, from_=-255, to=255, orient=HORIZONTAL)
        self.motor_left.pack(side=LEFT, fill=X, expand=True)

        motor_right_frame = Frame(root)
        motor_right_frame.pack(fill=X)
        Label(motor_right_frame, text="Motor Right").pack(side=LEFT)
        self.motor_right = Scale(motor_right_frame, from_=-255, to=255, resolution=1, orient=HORIZONTAL)
        self.motor_right.pack(side=LEFT, fill=X, expand=True)

        self.motor_right.config(to=10)

        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_motor_command_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Send Cont.', command=self.show_values, state="disabled").pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Send', command=self.show_values, state="disabled").pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Motor', command=self.stop_motors).pack(side=LEFT)
        

        enc_frame = Frame(root)
        enc_frame.pack(fill=X)

        self.enc_lbl = Label(enc_frame, text="Encoders: ")
        self.enc_lbl.pack(side=LEFT)
        self.motor_left_enc_lbl = Label(enc_frame, text="XXX")
        self.motor_left_enc_lbl.pack(side=LEFT)
        self.motor_right_enc_lbl = Label(enc_frame, text="XXX")
        self.motor_right_enc_lbl.pack(side=LEFT)

        speed_frame = Frame(root)
        speed_frame.pack(fill=X)

        self.spd_lbl = Label(enc_frame, text="Speed rev/s: ")
        self.spd_lbl.pack(side=LEFT)
        self.motor_left_spd_lbl = Label(enc_frame, text="XXX")
        self.motor_left_spd_lbl.pack(side=LEFT)
        self.motor_right_spd_lbl = Label(enc_frame, text="XXX")
        self.motor_right_spd_lbl.pack(side=LEFT)


        self.set_mode(True)


    def show_values(self):
        print(self.motor_left.get(),self.motor_right.get())

    def send_motor_command_once(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        msg.motor_left_req_rad_sec = 0.0
        msg.motor_left_req_rad_sec = 0.0
        self.publisher.publish(msg)

    def stop_motors(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        if(self.pwm_mode):
            msg.motor_left_req_rad_sec = float(self.motor_left.get())
            msg.motor_left_req_rad_sec = float(self.motor_left.get())
        else:
            msg.motor_left_req_rad_sec = float(self.motor_left.get()*2*math.pi)
            msg.motor_left_req_rad_sec = float(self.motor_left.get()*2*math.pi)

    def set_mode(self, mode):
        self.pwm_mode = mode

        if (self.pwm_mode):
            self.mode_lbl.config(text="Current Mode: PWM")
            self.mode_btn.config(text="Switch to Feedback Mode")
            self.slider_max_label.config(state="disabled")
            self.slider_max_val_box.config(state="disabled")
            self.max_val_update_btn.config(state="disabled")
        else:
            self.mode_lbl.config(text="Current Mode: Feedback")
            self.mode_btn.config(text="Switch to PWM Mode")
            self.slider_max_label.config(state="normal")
            self.slider_max_val_box.config(state="normal")
            self.max_val_update_btn.config(state="normal")

        self.update_scale_limits()

    def motor_velocitys_callback(self, motor_vels):
        motor_left_speed_revolution_per_sec = motor_vels.motor_left_rad_sec / (2*math.pi)
        motor_right_speed_revolution_per_sec = motor_vels.motor_left_rad_sec / (2*math.pi)
        self.motor_left_spd_lbl.config(text=f"{motor_left_speed_revolution_per_sec:.2f}")
        self.motor_right_spd_lbl.config(text=f"{motor_right_speed_revolution_per_sec:.2f}")

    def encoder_values_callback(self, encoder_vals):
        self.motor_left_enc_lbl.config(text=f"{encoder_vals.motor_left_enc_val}")
        self.motor_right_enc_lbl.config(text=f"{encoder_vals.motor_right_enc_val}")

    def switch_mode(self):
        self.set_mode(not self.pwm_mode)

    def update_scale_limits(self):
        if (self.pwm_mode):
            self.motor_left.config(from_ = -255, to = 255, resolution = 1)
            self.motor_right.config(from_ = -255, to = 255, resolution = 1)
        else :
            limit = float(self.slider_max_val_box.get())
            self.motor_left.config(from_ = -limit, to = limit, resolution = 0.1)
            self.motor_right.config(from_ = -limit, to = limit, resolution = 0.1)


    def update(self):
        self.tk.update()



def main(args = None):

    rclpy.init(args=args)

    serial_motor_gui = MotorGui()

    rate = serial_motor_gui.create_rate(20)
    while rclpy.ok():
        rclpy.spin_once(serial_motor_gui)
        serial_motor_gui.update()

    serial_motor_gui.destroy_node()
    rclpy.shutdown()



