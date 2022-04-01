import can
import cantools
import signal

import rospy
import os
import sys
from std_msgs.msg import Int8MultiArray

'''
1419 LCA11
CF_Lca_IndLeft 0 1 2 3
CF_Lca_IndRight 0 1 2 3
'''

class Can:
    def __init__(self):
        self.can_initialize()
        self.ros_initialize()
        self.mode = None

    def can_initialize(self):
        self.db = cantools.database.load_file(
            '/home/lattepanda/Documents/can/hyundai_can.dbc')
        self.bus = can.interface.Bus(
            bustype='socketcan', channel='can1', bitrate=500000)

        self.can_print = False

    def ros_initialize(self):
        rospy.init_node('can_record', anonymous=True)
        rospy.Subscriber("/BSD_check", Int8MultiArray, self.BSD_callback)

        self.ros_print = True

    def BSD_can_push(self):
        while True:
            message = can.Message(arbitration_id=1491, data=0)
            self.bus.send(message)

    def BSD_callback(self, msg):
        self.bsd_array = msg.data


if __name__ == "__main__":
    C = Can()
