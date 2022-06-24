from cereal.messaging import SubMaster
import numpy as np
import os
import rospy
import signal
import sys
import time
from std_msgs.msg import Bool, Int8, Int16MultiArray, Int16, Float32MultiArray

os.environ["ZMQ"] = "1"

class LaneCheck:
    def __init__(self):
        self.sm = None
        
        # self.sm = SubMaster(['modelV2', 'lateralPlan', 'carState'], addr=addr)

        self.temp = {'0': 0, '1': 0, '2': 0, '3': 0}

        rospy.init_node('laneline', anonymous=True)

        self.lane_state_pub = rospy.Publisher('/unstable_lane', Bool, queue_size=1)
        self.lane_warn_pub = rospy.Publisher('/lane_warn', Int8, queue_size=1)
        self.lkas_state = rospy.Publisher('/lkas', Bool, queue_size=1)
        self.op_fcw = rospy.Publisher('/op_fcw', Bool, queue_size = 1)
        self.ll_prob = rospy.Publisher('ll_prob', Float32MultiArray, queue_size = 1)

        rospy.Subscriber('/can_record', Int16MultiArray, self.can_record_callback)
       

        ##
        self.l_curvature = rospy.Publisher('/l_curvature', Int16, queue_size=1)
        self.r_curvature = rospy.Publisher('/r_curvature', Int16, queue_size=1)
        self.left_curvated = 0
        self.right_curvated = 0
        ##

        self.onLane = False
        self.warnLane = 0
        self.lkas = Bool()
        self.lkas.data = False
        self.vel = 0
        self.lane_prob = False
        self.edge = 0
    
    def can_record_callback(self, msg):
        self.vel = msg.data[5]

    def reconnect(self):
        addr = '192.168.101.100'
        self.sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
            'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal',
            'clocks', 'thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw',
            'cameraOdometry', 'carEvents', 'driverCameraState', 'driverMonitoringState'],
            addr=addr)

    def get_lane_lines(self):
        self.sm.update(0)
        if(time.time() - self.sm.rcv_time['modelV2'] > 1):
            self.lkas.data = True
            self.lkas_state.publish(self.lkas)
            #print('Fault')
            self.reconnect()
        else:
            self.lkas.data = False
            self.lkas_state.publish(self.lkas)
            if self.sm['modelV2']:               
                if self.sm['modelV2'].laneLines:
                    for i, _ in enumerate(self.sm['modelV2'].laneLines):
                        self.temp[str(i)] = _
                    x = self.temp['1'].x
                    line1s = self.temp['1'].y
                    line2s = self.temp['2'].y

                    # 2. Calculate Whether a Car is on the Lane or Not
                    self.onLane = False

                    #Delete On Lane Checker with Lane Width 

                    # lane_width= line2s[0]-line1s[0] 
                    # if (lane_width > 3.7):
                    #     #print("Lane 1 & 2's Width {} ".format(lane_width))
                    #     self.onLane = True

                    if self.lane_prob:
                        self.onLane = True
                    if (line1s[0]>-0.7):
                        #print("Lane 1 is Near 0")
                        self.onLane = True
                    if (line2s[0]<0.7):
                        #print("Lane 2 is Near 0")
                        self.onLane = True
                    if ((self.left_curvated < 90 or self.right_curvated < 90) and self.vel < 25):
                        self.onLane = True
                        #print("Large Curvature & Low Velocity")

                    # 3. Calculate Lane Departure
                    if self.sm['carState'].leftBlinker or self.sm['carState'].rightBlinker:
                        self.warnLane = 0
                    else:    
                        if (-0.9<line1s[0]<-0.75 or 0.75<line2s[0]<0.9):
                            self.warnLane = 1
                            #print("Lane Warning")
                        elif (-0.75<line1s[0] or 0.75>line2s[0]) or self.lane_prob or (self.left_curvated < 80 or self.right_curvated < 80):
                            self.warnLane = 2
                            #print("Lane Out")
                        else:
                            self.warnLane = 0
                            #print("Lane Stable")
                    
                     # 4. Calculate Curvature 
                    xx = np.array(x)[5:15] # for calculate forward lane (5~15)
                    lefty = np.array(line1s)[5:15]
                    righty = np.array(line2s)[5:15]
                    left_fit_cr = np.polyfit(xx, lefty, 2) # return poltnomial coefficient
                    right_fit_cr = np.polyfit(xx, righty, 2)
                    self.left_curvated = min(int(((1+(2*left_fit_cr[0]+left_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0])), 30000) # calculate curvature
                    self.right_curvated = min(int(((1+(2*right_fit_cr[0]+right_fit_cr[1])**2)**1.5)/np.absolute(2*right_fit_cr[0])), 30000)

                    #if(int(self.left_curvated) < 300):
                        #print("Can't Change Lanes Left Cur : {} ".format(int(self.left_curvated)))
                    #if(int(self.right_curvated) < 300):
                        #print("Can't Change Lanes Right Cur : {} ".format(int(self.right_curvated)))

            ###
            self.l_curvature.publish(self.left_curvated)
            self.r_curvature.publish(self.right_curvated)
            ###

            self.lane_state_pub.publish(self.onLane)
            self.lane_warn_pub.publish(self.warnLane)                   

    def get_fcw_events(self):
        FCW1 = False
        FCW2 = False
        FCW = False
        if self.sm['longitudinalPlan'] :
            FCW1 = self.sm['longitudinalPlan'].fcw
        if self.sm['modelV2']:
            FCW2 = self.sm['modelV2'].meta.hardBrakePredicted      
        if FCW1 or FCW2 :
            FCW = True
        self.op_fcw.publish(FCW)
    
    def get_lane_prob(self):
        if self.sm['modelV2'].laneLineProbs:
            prob = (self.sm['modelV2'].laneLineProbs[1]+self.sm['modelV2'].laneLineProbs[2])/2
            if prob < 0.2: # When there are no lanes 
                self.lane_prob = True
            else:
                self.lane_prob = False
            print(self.sm['modelV2'].laneLineProbs[1],self.sm['modelV2'].laneLineProbs[2])
    
    def get_edge(self):
        if self.sm['modelV2'].roadEdges:
            for i, _ in enumerate(self.sm['modelV2'].roadEdges):
                self.temp[str(i)] = _
            line2s = self.temp['1'].y
            self.edge = line2s[1]

# self.ll_prob.data = self.sm['modelV2'].laneLineProbs[1], self.sm['modelV2'].laneLineProbs[2]]
def signal_handler(sig, frame):
    #print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == "__main__":
    lc = LaneCheck()
    lc.reconnect()
    rate = rospy.Rate(5)
    #print("Lane Check Start !")
    while not rospy.is_shutdown():
        try:  
            signal.signal(signal.SIGINT, signal_handler)
            lc.get_lane_lines()
            lc.get_fcw_events()
            lc.get_lane_prob()
            lc.get_edge()
            rate.sleep()
        except Exception as e:
            #print(e)
            #print(type(e))
            exit()
