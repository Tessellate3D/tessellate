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

def rot_matrix(theta, axis):
	if axis == "Rx":
		#Rotation about x axis; yz plane yz plane unit circle
		Rx = np.matrix([[1, 0, 0], 
						[0, np.cos(theta), -np.sin(theta)], 
               			[0, np.sin(theta), np.cos(theta)]])
		return Rx
	elif axis == "Ry":
		#Rotation about y axis; xz plane xz plane unit circle
		Ry = np.matrix([[np.cos(theta), 0, np.sin(theta)], 
               			[0, 1, 0], 
               			[-np.sin(theta), 0, np.cos(theta)]])
		return Ry
	elif axis == "Rz":
		#Rotation about z axis; xy plane unit circle
		Rz = np.matrix([[np.cos(theta), -np.sin(theta), 0], 
               			[np.sin(theta), np.cos(theta), 0],
               			[0, 0, 1]])
		return Rz



### calculates new rotation states, changes arduino stepper motor
###
### PARAMATERS:
###				iterations - number of images
###				angle - rotation of camera pose/rotation of turntable
###				translation - change in height of camera
###				wait_time - time delay for arduino process
###				frame_path - frame destination
###				dest_path - path destination
###
### RETURNS -  Rotation matrices for extrinsics
def greg(iterations, angle, translation, wait_time, wait_stepper, image_path, dest_path):

	## SERIAL ##

	ser = serial.Serial('/dev/tty.usbmodem1421', 9600)  # open serial communication at 9600 baud to the arduino
	print("initializing...")
	time.sleep(5)  # wait for initialization of the serial communication to Arduino

	def arduino_message(rotation, translation, wait_time):
		###Turntable change

		rotation = str(-rotation) + "\n"
		translation = str(translation) + "\n"

		ser.write(rotation.encode())
		time.sleep(wait_time)

		###Camera height change
		ser.write(translation.encode())
		time.sleep(wait_time)
		### Add arduino call back message for debugging


	# ### List of rotation matrices for pose and orientation
	# rotations = []

	# ### matrix of height axis transformation
	# height = np.array([[0, 0, 0],
	# 				   [0, 0, 0],
	# 				   [0, 0, 1]])

	print("SERIAL UP: Beginning Iterations ")
	for i in range(iterations):

		print('---- ITERATION %d ----' % i)

		###message sent to arduino stepper motor; rotates turntable and adjust camera height
		###send negative of angle because of turntable design

		#capture current image
		
		arduino_message(angle, translation, wait_time)

		time.sleep(wait_stepper)



	#we travel iterations * translation up, and iterations * angle % 360 in angle
	#move back!!
	if i >= iterations - 1:
		print("===== MOVING DOWN =====")
		arduino_message(-(angle * iterations) % 360, - translation * iterations + H_OFFSET, wait_time)

		time.sleep(5)

	#release the stepper motors
	ser.write("release\n")
	time.sleep(wait_time)
	print('Done')

	# return rotations




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
image_path = '/home/greg/images/'
dest_path = '/home/greg/images/out/'



# if __name__ == '__main__':
# 	greg(img_num, theta, del_height, move_wait, stepper_wait, image_path, dest_path)

def connect_to_arduino(port):
	ser = serial.Serial(port, 9600)
	#ser = serial.Serial(port, 9600)  # open serial communication at 9600 baud to the arduino
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

def single_iteration(ser, angle=theta, translation=del_height, dest_path=dest_path, wait_time=move_wait, wait_stepper=stepper_wait):

	arduino_message(ser, angle, translation, wait_time)
	time.sleep(wait_stepper)
	return angle, translation

def reset_scanner(ser, current_angle, current_height, offset=H_OFFSET, wait_time=move_wait):
	print("===== MOVING DOWN =====")
	if current_height == 0:
		height = current_height
	else:
		height = - current_height + offset
	arduino_message(ser, -(current_angle) % 360, height, wait_time)
	time.sleep(5)

	#release the stepper motors
	ser.write("release\n".encode())
	time.sleep(wait_time)
	print('Scanner has been reset\n')







