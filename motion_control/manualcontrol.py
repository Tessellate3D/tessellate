import serial
import time
import os

from threading import Thread 

import sys
import select


ser = serial.Serial("/dev/ttyACM0", 9600)  # open serial communication at 9600 baud to the arduino
#ser = serial.Serial("/dev/cu.usbmodem1411", 9600)  # open serial communication at 9600 baud to the arduino
print("initializing...")
time.sleep(4)  # wait for initialization of the serial communication to Arduino


print("==================================================")
print("==== Enter command as space separated numbers ====")
print("====          <theta> <height>  or...         ====")
print("====                  release                 ====")
print("==================================================")

# def write():
# 	theta = 15
# 	for i in range(10):
# 		ser.write(str(theta*i) + "\n")
# 		time.sleep(2)

# def read():
# 	while True:
# 		for line in ser.read():
# 			print("RECEIVED: " + str(line))


# w = Thread(target=write)
# w.start()
# # read()
# w.join()

# print("DONE")

while True:
	rr, rw, in_error = select.select([sys.stdin],[],[], 0)

	for el in rr:
		if el is sys.stdin:
			line = sys.stdin.readline()
			elements = line.split()
			if len(elements)  != 2:
				if len(elements) == 1 and elements[0] == "release":
					ser.write(elements[0] + "\n")
				else:
					print("BAD ARGS")
			else:
				ser.write(elements[0] + "\n")
				ser.write(elements[1] + "\n")
				time.sleep(0.2)
				print("Sent.")
