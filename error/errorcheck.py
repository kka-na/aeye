import rospy
import os
import signal
import sys

from sensor_msgs.msg import CompressedImage, PointCloud2, Temperature
from visualization_msgs.msg import MarkerArray
from sbg_driver.msg import SbgEkfNav, SbgEkfEuler
from std_msgs.msg import Int8MultiArray, Float32MultiArray
# kana modify
from geometry_msgs.msg import Point

class Final:
    def __init__(self):

        rospy.init_node('sensor_check', anonymous = True)

        rospy.Subscriber('/gmsl_camera/dev/video0/compressed', CompressedImage, self.video0_Callback)
        rospy.Subscriber('/gmsl_camera/dev/video1/compressed', CompressedImage, self.video1_Callback)
        rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.LiDAR_points_Callback)
        rospy.Subscriber('/imu/temp', Temperature, self.IMU_temp_Callback)
        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.sbg_ekf_Callback)
        rospy.Subscriber('/Camera/Front60/od_bbox', Float32MultiArray, self.Video0_result_Callback)
        rospy.Subscriber('/pillar_marker_array', MarkerArray, self.LiDAR_result_Callback)
        #kana modify
        rospy.Subscriber('/gps_accuracy', Point, self.GPS_Acc_Callback)

        self.sensor_check = rospy.Publisher('/sensor_state', Int8MultiArray, queue_size = 1)
        self.system_check = rospy.Publisher('/system_state', Int8MultiArray, queue_size = 1)

        self.Video0_count = 0
        self.Video1_count = 0
        self.LiDAR_points_count = 0
        self.IMU_temp_count = 0
        self.Sbg_euler_count = 0
        self.Video0_result_count = 0
        self.LiDAR_result_count = 0
        #kana modify
        self.GPS_error_type = 0

    def video0_Callback(self, msg):
        self.Video0_count += 1
    
    def video1_Callback(self, msg):
        self.Video1_count += 1

    def LiDAR_points_Callback(self, msg):
        self.LiDAR_points_count += 1

    def IMU_temp_Callback(self, msg):
        self.IMU_temp_count += 1

    def sbg_ekf_Callback(self, msg):
        self.Sbg_euler_count += 1

    def Video0_result_Callback(self, msg):
        self.Video0_result_count += 1

    def LiDAR_result_Callback(self, msg):
        self.LiDAR_result_count += 1

    #kana modify
    def GPS_Acc_Callback(self, msg):
        if(msg.x == 0):
            self.GPS_error_type = 1
        elif(msg.y < 70):
            self.GPS_error_type = 2
        elif(msg.x == 1 and msg.y >= 70):
            self.GPS_error_type = 0

    

    def checker(self):
        sensor_state = Int8MultiArray() 
        system_state = Int8MultiArray()
        print("="*50)

        if self.Video0_count > 23:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])
        
        print("Narrow Camera : {}Hz".format(self.Video0_count))

        if self.Video1_count > 23:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])

        print("Wide Camera : {}Hz".format(self.Video1_count))

        if self.LiDAR_points_count > 6:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])

        print("LiDAR : {}Hz".format(self.LiDAR_points_count))

        if self.IMU_temp_count > 18:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])

        print("IMU : {}Hz".format(self.IMU_temp_count))

        if self.Sbg_euler_count > 18 and self.GPS_error_type == 0:
            sensor_state.data.extend([False])
        elif self.Sbg_euler_count <= 18 or self.GPS_error_type > 0:
            sensor_state.data.extend([True])

        '''
        #kana modify
        if self.GPS_error_type == 0 :
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])
        '''
        print("GPS Accuracy Error Type : {}".format(self.GPS_error_type))
        

        print("INS : {}Hz".format(self.Sbg_euler_count))

        if self.Video0_result_count > 20:
            system_state.data.extend([False])
        else:
            system_state.data.extend([True])

        print("Camera result : {}Hz".format(self.Video0_result_count))

        if self.LiDAR_result_count > 3:
            system_state.data.extend([False])
        else:
            system_state.data.extend([True])

        print("LiDAR result : {}Hz".format(self.LiDAR_result_count))

        


        self.Video0_count = 0
        self.Video1_count = 0
        self.LiDAR_points_count = 0
        self.IMU_temp_count = 0
        self.Sbg_euler_count = 0
        self.Video0_result_count = 0
        self.LiDAR_result_count = 0
        #kana modify
        self.GPS_error_type = 0

        self.talker(sensor_state, system_state)

    def talker(self,sensorstate, systemstate):
        print("="*50)
        print("Camera0, Camera1, LiDAR, IMU, INS, GPSAcc")
        print(sensorstate.data)

        print("Cam_result, LiDAR_result")
        print(systemstate.data)
        
        self.sensor_check.publish(sensorstate)
        self.system_check.publish(systemstate)

        self.sensor_check.data = []
        self.system_check.data = []

def signal_handler(sig, frame):
    print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == "__main__":
    mm = Final()
    print(" Check Start !")
    while not rospy.is_shutdown():
        try:
            rospy.sleep(1)
            signal.signal(signal.SIGINT, signal_handler)
            os.system('clear')
            mm.checker()
        except Exception as e:
            print(e)
            print(type(e))
            exit()




    
