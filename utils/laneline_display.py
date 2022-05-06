from cereal.messaging import SubMaster
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
os.environ["ZMQ"] = "1"

addr = '192.168.1.6'
sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
                            'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal',
                            'clocks', 'thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw',
                            'cameraOdometry', 'carEvents', 'driverCameraState',  'wideRoadCameraState', 'driverMonitoringState'],
               addr=addr)

temp = {'0': 0, '1': 0, '2': 0, '3': 0}

H = 1208//2
W =1928//2

while 1:
    sm.update(0)
    
    # 1. Road Camera streaming Test
    '''
    if sm.updated['roadCameraState']:
        bgr_raw = sm['roadCameraState'].image
        # print(len(bgr_raw))
        dat = np.frombuffer(bgr_raw, dtype=np.uint8).reshape(H, W, 3)
        avg = ticker - time.time() + avg / 2
        ticker = time.time()
        cv2.imshow('frame', dat)
        cv2.waitKey(1)
    '''

    if sm['modelV2']:
        if sm['modelV2'].laneLines:
            for i, _ in enumerate(sm['modelV2'].laneLines):
                temp[str(i)] = _

            x = np.uint32(temp['1'].x)
            line0s = temp['0'].y
            line1s = temp['1'].y
            line2s = temp['2'].y
            line3s = temp['3'].y

            # 2. Calculate Whether a Car is on the Lane or Not
            '''
            onLane = False
            if (line2s[0]-line1s[0] > 3.6):
                print("Lane 1 & 2's Width {} ".format(line2s[0]-line1s[0]))
                onLane = True
            if (line1s[0]>-0.7):
                print("Lane 1 is Near 0")
                onLane = True
            if (line2s[0]<0.7):
                print("Lane 2 is Near 0")
                onLane = True

            if(not onLane):
               print("Car is Keep in the Lane")
            else:
               print("Car is on the Lane")
            '''


            # 3. Calculate Lane Departure
            '''
            if (-0.9<line1s[0]<-0.75):
                print("Left Warning")
            elif (-0.75<line1s[0]):
                print("Left Out")
            else:
                print("Left Stable")

            if (0.75<line2s[0]<0.9):
                print("Right Warning")
            elif (0.75>line2s[0]):
                print("Right Out")
            else:
                print("Right Stable")
            '''


            # 4. Calculate Curvature 
            '''
            xx = np.array(x)[5:15] # for calculate forward lane (5~15)
            lefty = np.array(line1s)[5:15]
            righty = np.array(line2s)[5:15]
            left_fit_cr = np.polyfit(xx, lefty, 2) # return poltnomial coefficient
            right_fit_cr = np.polyfit(xx, righty, 2)
            left_curvated = ((1+(2*left_fit_cr[0]+left_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0]) # calculate curvature
            right_curvated = ((1+(2*right_fit_cr[0]+right_fit_cr[1])**2)**1.5)/np.absolute(2*right_fit_cr[0])

            if(int(left_curvated) < 300):
                print("Can't Change Lanes Left Cur : {} ".format(int(left_curvated)))
            if(int(right_curvated) < 300):
                print("Can't Change Lanes Right Cur : {} ".format(int(right_curvated)))
            '''


            # 5. Plotting Lanes 0,1,2,3
            plt.ion()
            plt.cla()
            plt.figure
            plt.xlim(-6, 6)
            plt.ylim(0, 50)
            animated_plot = plt.plot(
                line0s, x, 'bo', line1s, x, 'ro', line2s, x, 'ro', line3s, x, 'bo', linestyle='--')[0]

            plt.draw()
            plt.pause(0.1)
