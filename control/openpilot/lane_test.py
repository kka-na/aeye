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
        self.temp = {'0': 0, '1': 0, '2': 0, '3': 0}
        self.lane_width_estimate = 3.5
        self.lane_width_certainty = 1.0
        self.lane_width = 3.5

        self.lll_prob = 0.
        self.rll_prob = 0.
        self.d_prob = 0.

        self.lll_std = 0.
        self.rll_std = 0.

        self.v_ego = 0

        self.l_lane_change_prob = 0.
        self.r_lane_change_prob = 0.

        self.path_xyz = np.zeros((33,3))

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
            #print('Fault')
            self.reconnect()
        else:
            if self.sm['modelV2']:               
                if self.sm['modelV2'].laneLines:
                    for i, _ in enumerate(self.sm['modelV2'].laneLines):
                        self.temp[str(i)] = _

                    x = self.temp['1'].x
                    line0s = self.temp['0'].y
                    line1s = self.temp['1'].y
                    line2s = self.temp['2'].y
                    line3s = self.temp['3'].y

                    for i, _ in enumerate(self.sm['modelV2'].roadEdges):
                        self.temp[str(i)] = _

                    edge0s = self.temp['0'].y
                    edge1s = self.temp['1'].y

                    lane_width_ego = line2s[0]-line1s[0]
                    lane_width = line2s[15]-line1s[15]
                    #print("Left Lane : {} ".format(float(line1s[0])))
                    #print("Right Lane : {} ".format(float(line2s[0] )))
                    #print("Lane Width : {} ".format(float(lane_width)))
                    #print("Y : {}m {}m {}m {}m {}m".format(float(x[13]), float(x[14]), float(x[15]), float(x[16]), float(x[17])))
                    #print("Lane Width[0] : {} ".format(float(lane_width_ego)))
                    #print("="*50)

                    #13~17
                    #zipper merge
                    zipper_sum = 0
                    for i in range(13, 18):
                        zipper_sum += abs(edge1s[i]-line2s[i])
                    zipper_sum /= 5
                    print("Zipper SUM {}".format(zipper_sum))
            
                    # Lane Prob
                    self.lll_prob = self.sm['modelV2'].laneLineProbs[1]
                    self.rll_prob = self.sm['modelV2'].laneLineProbs[2]
                    #print("Left Lane Probability : {} ".format(float(self.lll_prob)))
                    #print("Right Lane Probability : {} ".format(float(self.rll_prob)))
                    d_prob = (self.lll_prob + self.rll_prob) - (self.lll_prob * self.rll_prob) # or - and => only one lane
                    #print("Only One Lane Probability : {} ".format(float(d_prob)))
                    #print("="*50)

                    # Lane STD
                    self.lll_std = self.sm['modelV2'].laneLineStds[1]
                    self.rll_std = self.sm['modelV2'].laneLineStds[2]
                    #print("Left Lane STD : {} ".format(float(self.lll_std)))
                    #print("Right Lane STD : {} ".format(float(self.rll_std)))
                    #print("="*50)

                    
                    self.path_xyz = np.column_stack([self.sm['modelV2'].position.x, self.sm['modelV2'].position.y, self.sm['modelV2'].position.z])
                    self.path_xyz[:, 1] -= -0.00
                    l_prob, r_prob = self.lll_prob, self.rll_prob
                    self.ll_x = np.array(x)
                    self.lll_y = np.array(line1s)
                    self.rll_y = np.array(line2s)
                    width_pts = self.rll_y - self.lll_y
                    prob_mods = []
                    for t_check in [0.0, 1.5, 3.0]:
                        width_at_t = np.interp(t_check * (self.v_ego + 7), self.ll_x, width_pts)
                        prob_mods.append(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
                    mod = min(prob_mods)
                    l_prob *= mod
                    r_prob *= mod

                    
                    # Reduce reliance on uncertain lanelines
                    l_std_mod = np.interp(self.lll_std, [.15, .3], [1.0, 0.0])
                    r_std_mod = np.interp(self.rll_std, [.15, .3], [1.0, 0.0])
                    l_prob *= l_std_mod
                    r_prob *= r_std_mod                    
                    self.lane_width_certainty += 0.05 * (l_prob * r_prob - self.lane_width_certainty)
                    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
                    self.lane_width_estimate += 0.005 * (current_lane_width - self.lane_width_estimate)
                    speed_lane_width = np.interp(self.v_ego, [0., 31.], [2.8, 3.5])
                    self.lane_width = self.lane_width_certainty * self.lane_width_estimate + \
                                        (1 - self.lane_width_certainty) * speed_lane_width

                    clipped_lane_width = min(4.0, self.lane_width)
                    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
                    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

                    self.d_prob = l_prob + r_prob - l_prob * r_prob      
                    #print("Lateral Plan")
                    #print("LaneWidth : {}".format(self.lane_width))
                    #print("D Probability : {}".format(self.d_prob))
                    #print("="*50)   

                    # Calculate Curvature
                    xx = np.array(x)[5:] # for calculate forward lane (5~15)
                    lefty = np.array(line1s)[5:]
                    righty = np.array(line2s)[5:]
                    left_fit_cr = np.polyfit(xx, lefty, 2) # return poltnomial coefficient
                    right_fit_cr = np.polyfit(xx, righty, 2)

                    self.left_curvated = ((1+(2*left_fit_cr[0]+left_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0]) # calculate curvature
                    self.right_curvated = ((1+(2*right_fit_cr[0]+right_fit_cr[1])**2)**1.5)/np.absolute(2*right_fit_cr[0])
                    #print("lane length : {},{},{}".format(xx.size, lefty.size, righty.size))
                    #print("Left Curvature : {} ".format(int(self.left_curvated)))
                    #print("Right Curvature : {} ".format(int(self.right_curvated)))
                    #print("="*50)       


                    # plotting
                    plt.ion()
                    plt.cla()
                    plt.figure
                    plt.xlim(-2.5, 7.5)
                    plt.ylim(0, 50)
                    animated_plot = plt.plot(edge0s, x, 'go', line0s, x, 'bo', line1s, x, 'ro', line2s, x, 'ro', line3s, x, 'bo', edge1s, x, 'go', linestyle='--')[0]

                    plt.draw()
                    plt.pause(0.1)

    def get_car_state(self):
        if self.sm['carState']:
            self.v_ego = self.sm['carState'].vEgo

        
def signal_handler(sig, frame):
    #print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == "__main__":
    lc = LaneCheck()
    lc.reconnect()
    #print("Lane Check Start !")
    while True:
        try:  
            signal.signal(signal.SIGINT, signal_handler)
            lc.get_lane_lines()
            lc.get_car_state()
            os.system('clear')
            time.sleep(0.2)
        except Exception as e:
            #print(e)
            #print(type(e))
            exit()
