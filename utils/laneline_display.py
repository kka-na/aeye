import numpy as np
import cv2
import matplotlib.pyplot as plt
from cereal.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType

from cereal.messaging import SubMaster

sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
                            'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal', 
                            'clocks','thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw', 
                            'cameraOdometry', 'carEvents', 'driverCameraState', 'driverMonitoringState'], addr='172.20.10.8')

temp = {'0' : 0, '1' : 0, '2' : 0, '3' : 0}

img = np.zeros((480, 640, 3), dtype='uint8')
imgff = None
num_px = 0
vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_RGB_ROAD, True)

while 1:
    # if not vipc_client.is_connected():
    #   vipc_client.connect(True)

    # rgb_img_raw = vipc_client.recv()

    # if rgb_img_raw is None or not rgb_img_raw.any():
    #   continue

    # num_px = len(rgb_img_raw) // 3
    # imgff_shape = (vipc_client.height, vipc_client.width, 3)

    # if imgff is None or imgff.shape != imgff_shape:
    #   imgff = np.zeros(imgff_shape, dtype=np.uint8)

    # imgff = np.frombuffer(rgb_img_raw, dtype=np.uint8).reshape((vipc_client.height, vipc_client.width, 3))
    # imgff = imgff[:, :, ::-1]  # Convert BGR to RGB
    
    #cv2.imshow("Test", imgff)
    
    sm.update(0)

    if sm['modelV2']:
        if sm['modelV2'].laneLines:
            for i, _ in enumerate(sm['modelV2'].laneLines):
                temp[str(i)] =  _
            
            x = np.uint32(temp['1'].x)
            
            line0s = temp['0'].y
            line1s = temp['1'].y
            line2s = temp['2'].y
            line3s = temp['3'].y

            bad = False

            if (line2s[0]-line1s[0] > 3.6):
                print("Lane 1 & 2's Width {} ".format(line2s[0]-line1s[0]))
                bad = True
            if (line1s[0]>-0.7):
                print("Lane 1 is Near 0")
                bad = True
            if (line2s[0]<0.7):
                print("Lane 2 is Near 0")
                bad = True
            
            if(not bad):
               print("State OK")
            else:
               print("State BAD")
            # print("left: ", line1s[0])
            # print("right: ", line2s[0])
            # if (-0.9<line1s[0]<-0.75):
            #     print("left warning")
            # elif (-0.75<line1s[0]):
            #     print("left out")
            # else:
            #     print("left stable")
            
            # if (0.75<line2s[0]<0.9):
            #     print("right warning")
            # elif (0.75>line2s[0]):
            #     print("right out")
            # else:
            #     print("right stable")


            plt.ion()
            plt.cla()
            plt.figure
            plt.xlim(-6, 6)
            plt.ylim(0, 50)
            animated_plot = plt.plot(line0s, x, 'bo', line1s, x, 'ro', line2s, x, 'ro', line3s, x, 'bo', linestyle='--')[0]

            plt.draw()
            plt.pause(0.1)