import zmq



def connect(port):
	context = zmq.Context()
	print("Connecting to pcl server at " + port)
	socket = context.socket(zmq.REQ)
	socket.connect(port)


def pcl_communicate(socket, radius, current_angle, current_height):
	message = str(radius) + " " + str(current_angle) + " " + str(current_height)
	print("Sending request %s …" % message)

	socket.send(message.encode('UTF-8'))
	socket.send("COMPUTE".encode('UTF-8'))
	response = socket.recv()

	return response

def shutdown():
	message = "STOP"
	print("Sending request %s …" % message)

	socket.send(message.encode('UTF-8'))
	response = socket.recv()
	
	return response

