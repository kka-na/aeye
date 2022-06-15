import rospy
from std_msgs.msg import String, Float32, Int8, Bool
from std_msgs.msg import Int8MultiArray, Int16MultiArray

'''
tor_record : 
[brake_pedal, accel_pedal, drvtq, e-stop, sensor error, system error, lane departure, AEB]
'''

class TOR_Record:
    def __init__(self):

        rospy.init_node('TOR_RECORD', anonymous=False)
        rospy.Subscriber('/can_switch', Int8MultiArray, self.can_switch_callback)
        rospy.Subscriber('/mode', Int8, self.mode_callback)
        rospy.Subscriber("/estop", Int8, self.estop_callback)
        rospy.Subscriber('/sensor_state', Int8MultiArray, self.sensor_state_callback)
        rospy.Subscriber('/system_state', Int8MultiArray, self.system_state_callback)
        rospy.Subscriber('/lane_warn', Int8, self.lane_warn_callback)
        #AEB sub ...

        self.button = rospy.Publisher('/mode_set', Int8, queue_size=1)
        self.tor_record = rospy.Publisher('/tor_record', Int8, queue_size=1)
        
        self.tor_record_array = Int8()
        self.tor_record_array.data = 0

        self.mode = 0
        self.can_switch_array = []
        self.estop = 0
        self.system_array = []
        self.sensor_array = []
        self.lane_warning = 0

        self.r = rospy.Rate(10) # 10hz 
        self.cnt = 0

        
    def can_switch_callback(self, msg):
        self.can_switch_array = msg.data
    def mode_callback(self, msg):
        self.mode = msg.data
    def estop_callback(self, msg):
        self.estop = msg.data
    def system_state_callback(self, msg):
        self.system_array = msg.data
    def sensor_state_callback(self, msg):
        self.sensor_array = msg.data
    def lane_warn_callback(self, msg):
        self.lane_warning = msg.data

    def publisher(self):
        self.tor_record.publish(self.tor_record_array)

    def record(self):
        while not rospy.is_shutdown():
            if self.cnt > 10:
                self.tor_record_array.data = 0
                self.cnt = 0

            if self.mode == 1 and self.can_switch_array[0] == 1:
                # self.button.publish(0)
                self.tor_record_array.data = 1

            elif self.mode == 1 and self.can_switch_array[1] == 1:
                # self.button.publish(0)
                self.tor_record_array.data = 2

            elif self.mode == 1 and self.can_switch_array[2] == 1:
                # self.button.publish(0)
                self.tor_record_array.data = 3

            elif self.mode == 1 and self.estop == 1:
                self.tor_record_array.data = 4

            elif self.mode == 1 and 1 in self.sensor_array:
                self.tor_record_array.data = 5

            elif self.mode == 1 and 1 in self.system_array:
                self.tor_record_array.data = 6

            elif self.mode == 1 and self.lane_warning == 2:
                self.tor_record_array.data = 7
            
            # elif self.mode == 1 and AEB!!!:
            #     self.tor_record_array.data = 8

            self.cnt += 1
            self.publisher()
            self.r.sleep()

if __name__ == '__main__':
    TOR = TOR_Record()
    TOR.record()
    rospy.spin()

