import rospy
import os
import signal
import sys

from sensor_msgs.msg import CompressedImage, PointCloud2
from visualization_msgs.msg import MarkerArray
from sbg_driver.msg import SbgEkfNav, SbgEkfEuler, SbgGpsPos
from std_msgs.msg import Int8MultiArray, Float32MultiArray
# kana modify
from geometry_msgs.msg import Point


class Final:
    def __init__(self):

        rospy.init_node('sensor_check', anonymous=True)

        rospy.Subscriber('/gmsl_camera/dev/video0/compressed',
                         CompressedImage, self.video0_Callback)
        rospy.Subscriber('/gmsl_camera/dev/video1/compressed',
                         CompressedImage, self.video1_Callback)
        rospy.Subscriber('/os_cloud_node/points', PointCloud2,
                         self.LiDAR_points_Callback)
        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.sbg_ekf_Callback)
        rospy.Subscriber('/sbg/gps_pos', SbgGpsPos, self.GPS_pos_Callback)
        rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.EKF_nav_Callback)
        rospy.Subscriber('/Camera/Front60/od_bbox',
                         Float32MultiArray, self.Video0_result_Callback)
        rospy.Subscriber('/pillar_marker_array', MarkerArray,
                         self.LiDAR_result_Callback)
        # kana modify
        self.gps_check = rospy.Publisher('/gps_accuracy', Point,  queue_size=1)
        self.sensor_check = rospy.Publisher(
            '/sensor_state', Int8MultiArray, queue_size=1)
        self.system_check = rospy.Publisher(
            '/system_state', Int8MultiArray, queue_size=1)

        self.Video0_count = 0
        self.Video1_count = 0
        self.LiDAR_points_count = 0
        self.Sbg_euler_count = 0
        self.Video0_result_count = 0
        self.LiDAR_result_count = 0
        # kana modify
        self.GPS_error_type = 0
        self.FIX_error_type = 0
        self.GPS_accuracy = 0

    def video0_Callback(self, msg):
        self.Video0_count += 1

    def video1_Callback(self, msg):
        self.Video1_count += 1

    def LiDAR_points_Callback(self, msg):
        self.LiDAR_points_count += 1

    def sbg_ekf_Callback(self, msg):
        self.Sbg_euler_count += 1

    def GPS_pos_Callback(self, msg):
        self.FIX_error_type = 1 if msg.diff_age > 500 else 0

    def EKF_nav_Callback(self, msg):
        self.GPS_error_type = 1 if msg.position_accuracy.x > 0.7 else 0
        self.GPS_accuracy = int(msg.position_accuracy.x * 100)

    def Video0_result_Callback(self, msg):
        self.Video0_result_count += 1

    def LiDAR_result_Callback(self, msg):
        self.LiDAR_result_count += 1

    def checker(self):
        sensor_state = Int8MultiArray()
        system_state = Int8MultiArray()
        gps_accuracy = Point()
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

        if self.GPS_error_type == 0 or self.FIX_error_type == 0:
            sensor_state.data.extend([False])
        else:
            sensor_state.data.extend([True])

        #print("GPS : {}Hz".format(self.IMU_temp_count))

        gps_accuracy.x = self.GPS_error_type
        gps_accuracy.y = self.GPS_accuracy

        if self.Sbg_euler_count > 18:
            sensor_state.data.extend([False])
        elif self.Sbg_euler_count <= 18:
            sensor_state.data.extend([True])

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
        self.Sbg_euler_count = 0
        self.Video0_result_count = 0
        self.LiDAR_result_count = 0
        # kana modify
        self.GPS_error_type = 0
        self.FIX_error_type = 0

        self.talker(sensor_state, system_state, gps_accuracy)

    def talker(self, sensorstate, systemstate, gpsaccuracy):
        print("="*50)
        print("Camera0, Camera1, LiDAR, GPS, INS")
        print(sensorstate.data)

        print("Cam_result, LiDAR_result")
        print(systemstate.data)

        self.sensor_check.publish(sensorstate)
        self.system_check.publish(systemstate)
        self.gps_check.publish(gpsaccuracy)

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
