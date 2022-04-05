#!/usr/bin/env python3

import rospy, time
from std_msgs.msg import Int8MultiArray, Int8, Bool
from geometry_msgs.msg import Vector3
from sbg_driver.msg import SbgEkfNav 

class ModeChanger:
    def __init__(self):
        isClear = True

        self.system_array = []
        self.sensor_array = []
        self.button = 0
        self.estop = 0
        self.pos_state = False # normal , True is Error
        self.acc = 0

        rospy.Subscriber('/sensor_state', Int8MultiArray, self.sensor_state_callback)
        rospy.Subscriber('/system_state', Int8MultiArray, self.system_state_callback)
        rospy.Subscriber('/mode_set', Int8, self.mode_set_callback)
        rospy.Subscriber('/estop', Int8, self.estop_callback)
        rospy.Subscriber('/acc', Int8, self.acc_callback)
        # rospy.Subscriber('/pos_state', Bool, self.pos_state_callback)
        # rospy.Subscriber('/sbg/ekf_nav',SbgEkfNav, self.ekf_nav)


        self.mode_pub = rospy.Publisher("/mode", Int8, queue_size=1)
        # self.gps = rospy.Publisher("/gps", Vector3, queue_size=1)

    # def ekf_nav(self, msg):
    #     location = Vector3()
    #     location.x = msg.latitude
    #     location.y = msg.longitude
    #     location.z = msg.altitude

    #     self.gps.publish(location)
        
    def system_state_callback(self, msg):
        self.system_array = msg.data

    def sensor_state_callback(self, msg):
        self.sensor_array = msg.data

    def mode_set_callback(self, msg):
        self.button = msg.data

    def estop_callback(self, msg):
        self.estop = msg.data

    def acc_callback(self, msg):
        self.acc = msg.data

    def pos_state_callback(self, msg):
        self.pos_state = msg.data # True of False

    def modePublisher(self):
        mode_msg = Int8()
        
        if self.estop == 1:
            #time.sleep(1)
            mode_msg.data = 0
            self.button = 0
            print(self.button, "E-STOP")
        # elif self.pos_state == True:
            # mode_msg.data = 0
            # self.buttion = 0
            # print(self.buttion, "Not position in k-city")
        elif self.button == 1 and (1 in self.system_array or 1 in self.sensor_array):
            # time.sleep(2)
            mode_msg.data = 0
            self.button = 0
            print(self.button, "Error")
        elif self.button == 1 and (not self.acc):
            # time.sleep(2)
            mode_msg.data = 0
            self.button = 0
            print(self.button, "ACC Error")
        elif self.button == 1:
            mode_msg.data = 1
            print(self.button, "autopilot")
        elif self.button == 0:
            mode_msg.data = 0
            print(self.button, "manual")
            
        self.mode_pub.publish(mode_msg)


def main():
    rospy.init_node('ModeChanger')
    chgmod = ModeChanger()
    rate = rospy.Rate(1)
    print("Ready to mode switch.")
    while not rospy.is_shutdown():
        # if # button clicked:
        chgmod.modePublisher()
        rate.sleep()
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
