from cereal.messaging import SubMaster
import os
import rospy
import signal
import sys
from std_msgs.msg import Bool, Int8

os.environ["ZMQ"] = "1"

class LaneCheck:
    def __init__(self):
        addr = '192.168.101.100'
        self.sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
            'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal',
            'clocks', 'thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw',
            'cameraOdometry', 'carEvents', 'driverCameraState', 'driverMonitoringState'],
            addr=addr)

        # self.sm = SubMaster(['modelV2', 'lateralPlan', 'carState'], addr=addr)

        self.temp = {'0': 0, '1': 0, '2': 0, '3': 0}

        rospy.init_node('laneline', anonymous=True)

        self.lane_state_pub = rospy.Publisher('/unstable_lane', Bool, queue_size=1)
        self.lane_warn_pub = rospy.Publisher('/lane_warn', Int8, queue_size=1)

        self.onLane = False
        self.warnLane = 0

       
    
    def get_lane_lines(self):
        self.sm.update(0)
        if self.sm['modelV2']:               
            if self.sm['modelV2'].laneLines:
                for i, _ in enumerate(self.sm['modelV2'].laneLines):
                    self.temp[str(i)] = _

                line1s = self.temp['1'].y
                line2s = self.temp['2'].y

                # 2. Calculate Whether a Car is on the Lane or Not
                self.onLane = False
                if (line2s[0]-line1s[0] > 3.5):
                    print("Lane 1 & 2's Width {} ".format(line2s[0]-line1s[0]))
                    self.onLane = True
                if (line1s[0]>-0.7):
                    print("Lane 1 is Near 0")
                    self.onLane = True
                if (line2s[0]<0.7):
                    print("Lane 2 is Near 0")
                    self.onLane = True

                if(not self.onLane):
                    print("STABLE MY LINE")

                # 3. Calculate Lane Departure
                if self.sm['carState'].leftBlinker or self.sm['carState'].rightBlinker:
                    self.warnLane = 0
                else:    
                    if (-0.9<line1s[0]<-0.75 or 0.75<line2s[0]<0.9):
                        self.warnLane = 1
                        print("Lane Warning")
                    elif (-0.75<line1s[0] or 0.75>line2s[0]):
                        self.warnLane = 2
                        print("Lane Out")
                    else:
                        self.warnLane = 0
                        print("Lane Stable")

        self.lane_state_pub.publish(self.onLane)
        self.lane_warn_pub.publish(self.warnLane)        
            

def signal_handler(sig, frame):
    print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == "__main__":
    lc = LaneCheck()
    rate = rospy.Rate(5)
    print("Lane Check Start !")
    while not rospy.is_shutdown():
        try:  
            signal.signal(signal.SIGINT, signal_handler)
            #os.system('clear')
            lc.get_lane_lines()
            rate.sleep()
        except Exception as e:
            print(e)
            print(type(e))
            exit()
