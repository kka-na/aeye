from cereal.messaging import SubMaster
import os
import rospy
import signal
import sys
import time
from std_msgs.msg import Bool, Int8
import numpy as np
import matplotlib.pyplot as plt

os.environ["ZMQ"] = "1"

class LaneCheck:
    def __init__(self):
        self.sm = None
        
        # self.sm = SubMaster(['modelV2', 'lateralPlan', 'carState'], addr=addr)

        self.temp = {'0': 0, '1': 0, '2': 0, '3': 0}


        self.left_curvated = 0
        self.right_curvated = 0


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
            # self.lkas.data = True
            # self.lkas_state.publish(self.lkas)
            print('Fault')
            self.reconnect()
        else:
            # self.lkas.data = False
            # self.lkas_state.publish(self.lkas)
            if self.sm['modelV2']:               
                if self.sm['modelV2'].laneLines:
                    for i, _ in enumerate(self.sm['modelV2'].laneLines):
                        self.temp[str(i)] = _

                    x = self.temp['1'].x
                    line0s = self.temp['0'].y
                    line1s = self.temp['1'].y
                    line2s = self.temp['2'].y
                    line3s = self.temp['3'].y

                    # 2. Calculate Whether a Car is on the Lane or Not
                    # self.onLane = False
                    # if (line2s[0]-line1s[0] > 3.7):
                    #     print("Lane 1 & 2's Width {} ".format(line2s[0]-line1s[0]))
                    #     # self.onLane = True
                    # if (line1s[0]>-0.7):
                    #     print("Lane 1 is Near 0")
                    #     # self.onLane = True
                    # if (line2s[0]<0.7):
                    #     print("Lane 2 is Near 0")
                    #     # self.onLane = True
                    # if self.left_curvated < 350 or self.right_curvated < 350:
                    #     # self.onLane = True
                    #     print("Large Curvature & Low Velocity")

                    # if(not self.onLane):
                        # print("STABLE MY LINE")

                    # 3. Calculate Lane Departure
                    # if self.sm['carState'].leftBlinker or self.sm['carState'].rightBlinker:
                        # self.warnLane = 0
                    # else:    
                    # if (-0.9<line1s[0]<-0.75 or 0.75<line2s[0]<0.9):
                    #     # self.warnLane = 1
                    #     print("Lane Warning")
                    # elif (-0.75<line1s[0] or 0.75>line2s[0]):
                    #     # self.warnLane = 2
                    #     print("Lane Out")
                    # else:
                    #     # self.warnLane = 0
                    #     print("Lane Stable")

                    lane_width = line2s[0]-line1s[0]
                    print("Left Lane : {} ".format(float(line1s[0])))
                    print("Right Lane : {} ".format(float(line2s[0] )))
                    print("Right Lane : {} ".format(float(lane_width)))
                    print("="*50)

                    # Lane Prob
                    lll_prob = self.sm['modelV2'].laneLineProbs[1]
                    rll_prob = self.sm['modelV2'].laneLineProbs[2]
                    print("Left Lane Probability : {} ".format(float(lll_prob)))
                    print("Right Lane Probability : {} ".format(float(rll_prob)))
                    d_prob = (lll_prob + rll_prob) - (lll_prob * rll_prob) # or - and => only one lane
                    print("Only One Lane Probability : {} ".format(float(rll_prob)))
                    print("="*50)

                    # Lane STD
                    lll_std = self.sm['modelV2'].laneLineStds[1]
                    rll_std = self.sm['modelV2'].laneLineStds[2]
                    print("Left Lane STD : {} ".format(float(lll_std)))
                    print("Right Lane STD : {} ".format(float(rll_std)))
                    print("="*50)

                    # Calculate Curvature
                    xx = np.array(x)[5:] # for calculate forward lane (5~15)
                    lefty = np.array(line1s)[5:]
                    righty = np.array(line2s)[5:]
                    left_fit_cr = np.polyfit(xx, lefty, 2) # return poltnomial coefficient
                    right_fit_cr = np.polyfit(xx, righty, 2)

                    self.left_curvated = ((1+(2*left_fit_cr[0]+left_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0]) # calculate curvature
                    self.right_curvated = ((1+(2*right_fit_cr[0]+right_fit_cr[1])**2)**1.5)/np.absolute(2*right_fit_cr[0])
                    print("lane length : {},{},{}".format(xx.size, lefty.size, righty.size))
                    print("Left Curvature : {} ".format(int(self.left_curvated)))
                    print("Right Curvature : {} ".format(int(self.right_curvated)))
                    print("="*50)

                    # if(int(self.left_curvated) < 300):
                    #     print("Can't Change Lanes Left Cur : {} ".format(int(self.left_curvated)))
                    # if(int(self.right_curvated) < 300):
                    #     print("Can't Change Lanes Right Cur : {} ".format(int(self.right_curvated)))

                    # plotting
                    plt.ion()
                    plt.cla()
                    plt.figure
                    plt.xlim(-6, 6)
                    plt.ylim(0, 50)
                    animated_plot = plt.plot(line0s, x, 'bo', line1s, x, 'ro', line2s, x, 'ro', line3s, x, 'bo', linestyle='--')[0]

                    plt.draw()
                    # plt.pause(0.1)

        
def signal_handler(sig, frame):
    print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == "__main__":
    lc = LaneCheck()
    lc.reconnect()
    # rate = rospy.Rate(5)
    print("Lane Check Start !")
    while True:
        try:  
            signal.signal(signal.SIGINT, signal_handler)
            os.system('clear')
            lc.get_lane_lines()
            time.sleep(0.2)
            # lc.get_fcw_events()
            # rate.sleep()
        except Exception as e:
            print(e)
            print(type(e))
            exit()
