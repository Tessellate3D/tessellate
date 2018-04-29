from flask import *
import os, sys
from threading import Thread
from time import sleep
import numpy as np
import serial
import time
import os
from move import *
from send_to_pcl import *

UPLOAD_FOLDER = 'meshes'
mesh_name = None

# Connect with Arduino
arduino_port = '/dev/tty.usbmodem1421'
ser = connect_to_arduino(arduino_port)

# PCL port
port = "tcp://localhost:5555"
pcl_socket = connect(port)


# Parameters
num_iters = 20
radius = 0.313151



def init_app(app):
    "Initialize app object. Create upload folder if it does not exist."
    if not os.path.isabs(app.config['UPLOAD_FOLDER']):
        folder = os.path.join(os.getcwd(), app.config['UPLOAD_FOLDER'])
        app.config['UPLOAD_FOLDER'] = folder
    if not os.path.exists(app.config['UPLOAD_FOLDER']):
        os.makedirs(app.config['UPLOAD_FOLDER'])

app = Flask(__name__)
app.config.from_object(__name__)
init_app(app)


running = False
current_angle = 0
current_height = 0


def reset():
	global current_angle
	global current_height

	reset_scanner(ser, current_angle, current_height)
	current_angle = 0
	current_height = 0

def scan():
    global running
    global current_angle
    global current_height
    current = 0

    while current <= num_iters and running:
    	print("Executing iteration " + str(current))

    	### Move Arduino to next state
    	temp_angle, temp_height = single_iteration(ser)

    	### Update current positions
    	current_angle += temp_angle
    	current_height += temp_height

    	### Call PCL library with updated paramaters
    	pcl_send(pcl_socket, radius, current_angle, current_height)
    	pcl_compute(pcl_socket)

    	current += 1
    reset()


def thread_run():
    t = Thread(target=scan)
    t.start()
    return "Processing"




@app.route('/static/<path:path>')
def send_static(path):
    return send_from_directory('static', path)

@app.route("/")
def index():
	return render_template('index.html')


@app.route("/start/", methods=['POST'])
def start():
	global mesh_name
	
	mesh_name = request.form['mesh_name']
	if not mesh_name:
		return render_template('index.html', error='Bad Mesh Name!')
	mesh_name = mesh_name + ".stl" #add file extension



	#SCANNING CODE
	print("Starting Scan" + "\n")
	global running
	running = True
	thread_run()

	print("Finished Scanning" + "\n")


	print("Uploading Mesh" + "\n")


	return render_template('index.html')

@app.route("/stop/", methods=['POST'])
def stop():
	#PAUSE CODE
	global running
	running = False

	stop = request.get_data()
	if not stop:
		print("User wants to reset")
	reset()
	time.sleep(2)
	return render_template('index.html', response=stop)




@app.route("/download/", methods=['POST'])
def download():
    # return render_template('index.html')
    if not mesh_name:
    	return render_template('index.html', error='No Object Scanned Yet!')

    uploads = os.path.join(current_app.root_path, app.config['UPLOAD_FOLDER'])
    return send_from_directory(directory=uploads, filename=mesh_name, as_attachment=True)




