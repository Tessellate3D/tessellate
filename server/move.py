import numpy as np
import serial
import time
import os

from threading import Thread

def deg_to_rad(deg):
    rad = (np.pi / 180) * deg
    return rad


def get_x_coord(r, ang):
    x = r * np.cos(deg_to_rad(ang))
    return x


def get_y_coord(r, ang):
    y = r * np.sin(deg_to_rad(ang))
    return y



###Rotation parameters
theta = 18
del_height = 5 ##5mm ###for changing scaling, adjust code in arduino stepper file
img_num = 20
### 


# CONSTANTS
move_wait = 0.03
# seconds to wait for the stepper to turn
stepper_wait = 1

H_OFFSET = 2 #mm from base to prevent crashing
# frame_wait = 2  # seconds to wait for the webcam to capture a frame
# iterations = 32

###File input/output destination
dest_path = '/home/greg/images/out/'



def connect_to_arduino(port):
	ser = serial.Serial(port, 9600)  # open serial communication at 9600 baud to the arduino
	print("initializing...")
	time.sleep(5)  # wait for initialization of the serial communication to Arduino
	return ser

def arduino_message(ser, rotation=theta, translation=del_height, wait_time=move_wait):
	###Turntable change

	rotation = str(-rotation) + "\n"
	translation = str(translation) + "\n"

	ser.write(rotation.encode())
	time.sleep(wait_time)

	###Camera height change
	ser.write(translation.encode())
	time.sleep(wait_time)

def single_iteration(ser, angle=theta, translation=del_height, wait_time=move_wait, wait_stepper=stepper_wait):

	arduino_message(ser, angle, translation, wait_time)
	time.sleep(wait_stepper)
	return angle, translation

def single_iteration2(ser, counter, angle=theta, translation=del_height, wait_time=move_wait, wait_stepper=stepper_wait):
        if counter == 2:
             arduino_message(ser, angle, translation, wait_time)
        else:
             arduino_message(ser, angle, 0, wait_time)
        time.sleep(wait_stepper)
        return angle, translation

def reset_scanner(ser, current_angle, current_height, offset=H_OFFSET, wait_time=move_wait):
	print("===== MOVING DOWN =====")
	if current_height == 0:
		height = current_height
	else:
		height = - current_height + offset
	arduino_message(ser, -(current_angle) % 360, height, wait_time)
	time.sleep(2)

	#release the stepper motors
	ser.write("release\n".encode())
	time.sleep(wait_time)
	print('Scanner has been reset\n')







