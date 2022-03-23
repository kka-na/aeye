import rospy
import sys
import signal
import serial
import time
import math
import os

from geometry_msgs.msg import Point

#accuracy threshold 70% 
#7.8m -> 95% reliablity ( from USA )
#1.56m -> 1% Error, 99% Reliability ( just compare radius ( not width of circle ) )


class GPSAccuracy:
	def __init__(self):
		rospy.init_node('gps_check', anonymous = True)
		self.pub_gps_accuracy = rospy.Publisher('/gps_accuracy', Point, queue_size=1)
		self.serial_gps = serial.Serial("/dev/ttyACM0", baudrate=38400)
		self.send_gps_accuracy = Point()
		self.fixed = 0
		self.position_err = 0
		self.accuracy = 0
		self.pdop = 1.8
		self.range_err = 1.8

	def checker(self):
		line = self.serial_gps.readline()
		data = line.split(',')
		if(data[0] == "$GNGGA"):
			fix_param = int(data[6])
			if(fix_param == 0 or fix_param == 6):
				self.fixed = 0
			elif(fix_param == 1 or fix_param == 2):
				self.fixed = 1
			else:
				self.fixed = 0
		if(data[0] == "$GNGSA"):
			self.pdop = float(data[15])
		if(data[0]=="$GNGST"):
			alt = data[8].split("*")
			lat_err = float(data[6])
			ln_err = float(data[7])
			alt_err = float(alt[0])
			self.range_err = lat_err + ln_err + alt_err
		self.calcs()
		self.publish()
		self.printAcc()

	    
	def calcs(self):
	    self.position_err = float(self.range_err * self.pdop)
	    self.accuracy = round((100-(self.position_err/1.56)),2)
	    if(self.accuracy < 0 ):
	        self.accuracy = 0
	    elif(self.accuracy > 100):
	        self.accuracy = 100

	
	def publish(self):
		self.send_gps_accuracy.x = self.fixed
		self.send_gps_accuracy.y = self.accuracy
		self.pub_gps_accuracy.publish(self.send_gps_accuracy)

	
	def printAcc(self):
		print("GPS Fixed State = {}".format(self.fixed))
		print("PDOP = {} Range_Error = {}".format(self.pdop, self.range_err))
		print("Total Position Err = {}".format(self.position_err))
		print("Accuracy = {}% ".format(self.accuracy))


def signal_handler(sig,frame):
	sys.exit(0)

if __name__ == "__main__":
	ga = GPSAccuracy()
	while not rospy.is_shutdown():
		try:
			signal.signal(signal.SIGINT, signal_handler)
			#os.system('clear')
			ga.checker()
		except Exception as e:
			print(e)
			exit()

