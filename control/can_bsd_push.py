import can
import cantools

import rospy
import time
from std_msgs.msg import Int8MultiArray

'''
1060 aeye
CF_Custom_BSDLeft 0 1
CF_Custom_BSDRight 0 1
'''

class Can:
    def __init__(self):
        self.can_initialize()
        self.ros_initialize()

    def can_initialize(self):
        self.db = cantools.database.load_file(
            '/home/lattepanda/Documents/can/hyundai_can.dbc')
        #self.bus1 = can.interface.Bus(
            #bustype='socketcan', channel='can1', bitrate=500000)
        self.bus1 = can.ThreadSafeBus(
            interface='socketcan', channel='can1', bitrate=500000)

    def ros_initialize(self):
        rospy.Subscriber("/BSD_check", Int8MultiArray, self.BSD_callback)
        self.bsd_array = [0, 0]

    def can_frame_planning(self):
        if self.bsd_array == [0, 0]:
            bsd_data = self.db.encode_message('aeye', {'custom_bsd_left':0, 'custom_bsd_right':0})
            self.bsd_can_msg = can.Message(arbitration_id=1060, data=bsd_data)
        elif self.bsd_array == [0, 1]:
            bsd_data = self.db.encode_message('aeye', {'custom_bsd_left':0, 'custom_bsd_right':1})
            self.bsd_can_msg = can.Message(arbitration_id=1060, data=bsd_data)
        elif self.bsd_array == [1, 0]:
            bsd_data = self.db.encode_message('aeye', {'custom_bsd_left':1, 'custom_bsd_right':0})
            self.bsd_can_msg = can.Message(arbitration_id=1060, data=bsd_data)
        elif self.bsd_array == [1, 1]:
            bsd_data = self.db.encode_message('aeye', {'custom_bsd_left':1, 'custom_bsd_right':1})
            self.bsd_can_msg = can.Message(arbitration_id=1060, data=bsd_data)

    def bsd2can(self):
        self.can_frame_planning()
        self.bus1.send(self.bsd_can_msg)

    def BSD_callback(self, msg):
        self.bsd_array = msg.data


def main():
    rospy.init_node('BSD2CAN', anonymous=True)
    c = Can()
    rate = rospy.Rate(10)
    print("Ready to BSD2CAN")
    while not rospy.is_shutdown():
        c.bsd2can()
        rate.sleep()
    
    rospy.spin()


if __name__ == "__main__":
    main()
