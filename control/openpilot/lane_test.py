from cereal.messaging import SubMaster
import os
import signal
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

os.environ["ZMQ"] = "1"


class LaneCheck:
    def __init__(self):
        self.sm = None

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

        self.line_dict = {'0': 0, '1': 0, '2': 0, '3': 0}
        self.edge_dict = {'0': 0, '1': 0}

    def reconnect(self):
        addr = '192.168.101.100'
        self.sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
                             'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal',
                             'clocks', 'thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw',
                             'cameraOdometry', 'carEvents', 'driverCameraState', 'driverMonitoringState'],
                            addr=addr)

    def get_lane_lines(self):
        self.sm.update(0)
        md = self.sm['modelV2']
        if(time.time() - self.sm.rcv_time['modelV2'] > 1):
            self.reconnect()
        else:
            if md:
                # Print MetaData
                '''
                struct MetaData {
                    engagedProb @0 :Float32;
                    desirePrediction @1 :List(Float32);
                    desireState @5 :List(Float32);
                    disengagePredictions @6 :DisengagePredictions;
                    hardBrakePredicted @7 :Bool;

                    # deprecated
                    brakeDisengageProbDEPRECATED @2 :Float32;
                    gasDisengageProbDEPRECATED @3 :Float32;
                    steerOverrideProbDEPRECATED @4 :Float32;
                }

                struct DisengagePredictions {
                    t @0 :List(Float32);
                    brakeDisengageProbs @1 :List(Float32);
                    gasDisengageProbs @2 :List(Float32);
                    steerOverrideProbs @3 :List(Float32);
                    brake3MetersPerSecondSquaredProbs @4 :List(Float32);
                    brake4MetersPerSecondSquaredProbs @5 :List(Float32);
                    brake5MetersPerSecondSquaredProbs @6 :List(Float32);
                }

                enum Desire {
                    none @0;
                    turnLeft @1;
                    turnRight @2;
                    laneChangeLeft @3;
                    laneChangeRight @4;
                    keepLeft @5;
                    keepRight @6;
                }

                '''

                desirePrediction_turnLeft = list(md.meta.desirePrediction)[4:8]
                desirePrediction_laneChangeLeft = list(md.meta.desirePrediction)[16:20]
                print("Desire Prediction Turn Left: {}\n".format(desirePrediction_turnLeft))
                print("Desire Prediction Lane Change Left: {}\n".format(desirePrediction_laneChangeLeft))
                '''
                brakeDisengageProbs = md.meta.disengagePredictions.brakeDisengageProbs
                gasDisengageProbs = md.meta.disengagePredictions.gasDisengageProbs
                steerOverrideProbs = md.meta.disengagePredictions.steerOverrideProbs
                print("Brake Disengage Probs : {}\n".format(brakeDisengageProbs))
                print("Gas Disengage Probs : {}\n".format(gasDisengageProbs))
                print("Steer Override Probs : {}\n".format(steerOverrideProbs))
                '''
                # LaneLines
                for i, _ in enumerate(md.laneLines):
                    self.line_dict[str(i)] = _
                x = self.line_dict['1'].x
                line0s = self.line_dict['0'].y
                line1s = self.line_dict['1'].y
                line2s = self.line_dict['2'].y
                line3s = self.line_dict['3'].y

                # Road Edges
                for i, _ in enumerate(md.roadEdges):
                    self.edge_dict[str(i)] = _
                edge0s = self.edge_dict['0'].y
                edge1s = self.edge_dict['1'].y

                # Lane Prob
                self.lll_prob = md.laneLineProbs[1]
                self.rll_prob = md.laneLineProbs[2]
                # or - and => only one lane
                d_prob = (self.lll_prob + self.rll_prob) - \
                    (self.lll_prob * self.rll_prob)
                '''
                print("Lane Probability")
                print("Left Lane Probability : {} ".format(float(self.lll_prob)))
                print("Right Lane Probability : {} ".format(float(self.rll_prob)))
                print("Only One Lane Probability : {} ".format(float(d_prob)))
                print("="*50)
                '''

                # Lane STD
                self.lll_std = md.laneLineStds[1]
                self.rll_std = md.laneLineStds[2]
                '''
                print("Lane Standard Deviation")
                print("Left Lane STD : {} ".format(float(self.lll_std)))
                print("Right Lane STD : {} ".format(float(self.rll_std)))
                print("="*50)
                '''

                # Lane Width of Y = 42 m"
                lane_width = line2s[15]-line1s[15]
                '''
                print("Lane Width of y={}m : {} ".format(float(lane_width)))
                print("="*50)
                '''

                # From lane_planner calc current width
                l_prob, r_prob = self.lll_prob, self.rll_prob
                self.ll_x = np.array(x)
                self.lll_y = np.array(line1s)
                self.rll_y = np.array(line2s)
                width_pts = self.rll_y - self.lll_y
                prob_mods = []
                for t_check in [0.0, 1.5, 3.0]:
                    width_at_t = np.interp(
                        t_check * (self.v_ego + 7), self.ll_x, width_pts)
                    prob_mods.append(
                        np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
                mod = min(prob_mods)
                l_prob *= mod
                r_prob *= mod

                # Reduce reliance on uncertain lanelines
                l_std_mod = np.interp(self.lll_std, [.15, .3], [1.0, 0.0])
                r_std_mod = np.interp(self.rll_std, [.15, .3], [1.0, 0.0])
                l_prob *= l_std_mod
                r_prob *= r_std_mod
                self.lane_width_certainty += 0.05 * \
                    (l_prob * r_prob - self.lane_width_certainty)
                current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
                self.lane_width_estimate += 0.005 * \
                    (current_lane_width - self.lane_width_estimate)
                speed_lane_width = np.interp(self.v_ego, [0., 31.], [2.8, 3.5])
                self.lane_width = self.lane_width_certainty * self.lane_width_estimate + \
                    (1 - self.lane_width_certainty) * speed_lane_width
                self.d_prob = l_prob + r_prob - l_prob * r_prob
                '''      
                print("Lateral Plan")
                print("LaneWidth : {}".format(self.lane_width))
                print("D Probability : {}".format(self.d_prob))
                print("="*50)   
                '''

                # plotting
                plt.ion()
                plt.cla()
                plt.figure
                plt.xlim(-2.5, 7.5)
                plt.ylim(0, 50)
                plt.plot(edge0s, x, 'go', line0s, x, 'bo', line1s, x, 'ro', line2s,
                         x, 'ro', line3s, x, 'bo', edge1s, x, 'go', linestyle='--')
                # plt.draw()
                # plt.pause(0.1)

    def get_car_state(self):
        if self.sm['carState']:
            self.v_ego = self.sm['carState'].vEgo


def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == "__main__":
    lc = LaneCheck()
    lc.reconnect()
    while True:
        try:
            signal.signal(signal.SIGINT, signal_handler)
            lc.get_lane_lines()
            lc.get_car_state()
            os.system('clear')
            time.sleep(0.2)
        except Exception as e:
            print(e)
            print(type(e))
            exit()
