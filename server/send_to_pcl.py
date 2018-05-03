import zmq
import numpy as np


def connect(port):
	context = zmq.Context()
	print("Connecting to pcl server at " + port)
	socket = context.socket(zmq.REQ)
	socket.connect(port)
	return socket 

def deg_to_rad(deg):
    rad = (np.pi / 180) * deg
    return rad



def pcl_send(socket, radius, current_angle, current_height):
	current_angle = deg_to_rad(current_angle)
	current_height = current_height/1000
	message = str(radius) + "," + str(current_angle) + "," + str(current_height)
	print("Sending request %s …" % message)
	socket.send(message.encode())
	response = socket.recv()
	print("Response from PCL: " + response)
	return response

def pcl_compute(socket):
	socket.send("COMPUTE".encode())
	response = socket.recv()
	print("Response from PCL: " + response)
	return response

def shutdown(socket):
	message = "STOP"
	print("Sending request %s …" % message)

	socket.send(message.encode())
	response = socket.recv()
	print("Response from PCL: " + response)
	return response

