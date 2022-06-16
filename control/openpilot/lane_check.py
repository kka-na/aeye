from cereal.messaging import SubMaster
import numpy as np
#import cv2
import matplotlib.pyplot as plt
import os
import rospy
from std_msgs.msg import Bool, Int8

os.environ["ZMQ"] = "1"

# addr = '172.20.10.8'
# addr = '192.168.1.6'
addr = '192.168.101.100'
sm = SubMaster(['carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'carParams',
    'liveTracks', 'modelV2', 'liveParameters', 'lateralPlan', 'sendcan', 'gpsLocationExternal',
    'clocks', 'thumbnail', 'roadCameraState', 'driverState', 'procLog', 'ubloxGnss', 'ubloxRaw',
    'cameraOdometry', 'carEvents', 'driverCameraState', 'driverMonitoringState'],
    addr=addr)

temp = {'0': 0, '1': 0, '2': 0, '3': 0}

H = 1208//2
W =1928//2


lane_state_pub = rospy.Publisher('/unstable_lane', Bool, queue_size=1)
lane_warn_pub = rospy.Publisher('/lane_warn', Int8, queue_size=1)

#for Openpilot Error Checking
op_null_pub = rospy.Publisher('/op_null', Bool, queue_size=1)
op_null=Bool()
op_null.data = False

rospy.init_node('laneline', anonymous=True)
rate = rospy.Rate(5)

onLane = False
warnLane = 0
#while 1:
while not rospy.is_shutdown():
    sm.update(0)
    # print("ok?")
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
        #print("state_1")
        #print(sm['modelV2'].laneLines)
        if sm['modelV2'].laneLines:
            #print("state_2")
            for i, _ in enumerate(sm['modelV2'].laneLines):
                temp[str(i)] = _

            x = np.uint32(temp['1'].x)
            line0s = temp['0'].y
            line1s = temp['1'].y
            line2s = temp['2'].y
            line3s = temp['3'].y

            # 2. Calculate Whether a Car is on the Lane or Not
            
            onLane = False
            if (line2s[0]-line1s[0] > 3.5):
                print("Lane 1 & 2's Width {} ".format(line2s[0]-line1s[0]))
                onLane = True
            if (line1s[0]>-0.7):
                print("Lane 1 is Near 0")
                onLane = True
            if (line2s[0]<0.7):
                print("Lane 2 is Near 0")
                onLane = True

            if(not onLane):
               print("STABLE MY LINE")
            # else:
            #    print("Car is on the Lane")
            


            # 3. Calculate Lane Departure
            if sm['carState'].leftBlinker or sm['carState'].rightBlinker:
                warnLane = 0
            else:    
                if (-0.9<line1s[0]<-0.75 or 0.75<line2s[0]<0.9):
                    warnLane = 1
                    print("Lane Warning")
                elif (-0.75<line1s[0] or 0.75>line2s[0]):
                    warnLane = 2
                    print("Lane Out")
                else:
                    warnLane = 0
                    print("Lane Stable")

                # if ():
                #     warnLane = 1
                #     print("Right Warning")
                # elif ():
                #     warnLane = 2
                #     print("Right Out")
                # else:
                #     warnLane = 0
                #     print("Right Stable")
        #If Lanelines doesn't work, 
        else: 
            # There is some Error
            op_null.data = True
            op_null_pub.publish(op_null)

    lane_state_pub.publish(onLane)
    lane_warn_pub.publish(warnLane)
    rate.sleep()
            
