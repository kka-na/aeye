import rospy
from std_msgs.msg import String, Float32, Int8, Bool
from std_msgs.msg import Int8MultiArray, Int16MultiArray

'''
tor_record : 
[drvtq, brake_pedal, accel_pedal, ui_button, e-stop, sensor error, system error, lane departure, AEB]
'''

class TOR_Record:
    def __init__(self):

        rospy.init_node('TOR_RECORD', anonymous=False)
        rospy.Subscriber('/can_switch', Int8MultiArray, self.can_switch_callback)
        rospy.Subscriber('/mode', Int8, self.mode_callback)
        rospy.Subscriber('/mode_set', Int8, self.mode_set_callback)
        rospy.Subscriber("/estop", Int8, self.estop_callback)
        rospy.Subscriber('/sensor_state', Int8MultiArray, self.sensor_state_callback)
        rospy.Subscriber('/system_state', Int8MultiArray, self.system_state_callback)
        rospy.Subscriber('/lane_warn', Int8, self.lane_warn_callback)
        #AEB sub ...

        self.tor_record = rospy.Publisher('/tor_record', Int8, queue_size=1)
        
        self.tor_record_array = Int8()
        self.tor_record_array.data = 0
        # [drvtq, brake_pedal, accel_pedal, ui_button, e-stop, sensor error, system error, lane departure, AEB]

        self.mode = 0
        self.can_switch_array = []
        self.button = 0
        self.estop = 0
        self.system_array = []
        self.sensor_array = []
        self.lane_warning = 0

        self.r = rospy.Rate(10) # 10hz 
        self.cnt = 0

        
    def can_switch_callback(self, msg):
        self.can_switch_array = msg.data
    def mode_callback(self, msg):
        self.mode = msg.data
    def mode_set_callback(self, msg):
        self.button = msg.data
    def estop_callback(self, msg):
        self.estop = msg.data
    def system_state_callback(self, msg):
        self.system_array = msg.data
    def sensor_state_callback(self, msg):
        self.sensor_array = msg.data
    def lane_warn_callback(self, msg):
        self.lane_warning = msg.data

    def publisher(self):
        self.tor_record.publish(self.tor_record_array)

    def record(self):
        while not rospy.is_shutdown():
            if self.cnt > 100:
                self.tor_record_array.data = 0
                self.cnt = 0

            if self.mode == 1 and self.can_switch_array[0] == 1:
                self.tor_record_array.data = 1

            elif self.mode == 1 and self.can_switch_array[1] == 1:
                self.tor_record_array.data = 2

            elif self.mode == 1 and self.can_switch_array[2] == 1:
                self.tor_record_array.data = 3

            elif self.mode == 1 and self.button == 0:
                self.tor_record_array.data = 4

            elif self.mode == 1 and self.estop == 1:
                self.tor_record_array.data = 5

            elif self.mode == 1 and 1 in self.sensor_array:
                self.tor_record_array.data = 6

            elif self.mode == 1 and 1 in self.system_array:
                self.tor_record_array.data = 7

            elif self.mode == 1 and lane_warning == 2:
                self.tor_record_array.data = 8

            self.cnt += 1
            self.publisher()
            self.r.sleep()

        
    def calculate_can(self):
        record = self.can_record_data.data
        switch = [0, 0, 0]

        record[0] = int(self.brake_pedal)
        record[1] = int(self.accel_pedal)
        record[2] = int(self.sas_angle)
        record[3] = self.gear_map[str(self.rcv_gear)]
        record[4] = int(self.rpm)
        record[5] = int(self.rcv_wheel_speed)


        if self.brake_pedal > 10 and self.mode == 1:
            switch[0] = 1
        elif self.accel_pedal > 10 and self.mode == 1:
            switch[1] = 1
        elif abs(self.drvtq) > 160 and self.mode == 1:
            switch[2] = 1

        if 1 in switch:
            self.mode_pub.publish(0)

        return record, switch

    def set_sw_state(self, current, target):
        # if(self.sas_angle > 10):
        #     target = min(target, target - int(abs(self.sas_angle)/20))
        if current < target:
            return 1
        elif current > target:
            return 2
        elif current == target:
            return 0

    def send_clu(self):
        cnt = 0
        _cnt = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
        data = {'CF_Clu_CruiseSwState': 0.0, 'CF_Clu_CruiseSwMain': 0.0, 'CF_Clu_SldMainSW': 0.0, 'CF_Clu_ParityBit1': 0.0, 'CF_Clu_VanzDecimal': 0.0, 'CF_Clu_Vanz': 0.0, 'CF_Clu_SPEED_UNIT': 0.0, 'CF_Clu_DetentOut': 1.0, 'CF_Clu_RheostatLevel': 12.0, 'CF_Clu_CluInfo': 0.0, 'CF_Clu_AmpInfo': 0.0, 'CF_Clu_AliveCnt1': 0.0}
        while 1:
            try:
                data['CF_Clu_AliveCnt1'] = _cnt[cnt%30] #INT OVERFLOW~!
                data['CF_Clu_Vanz'] = self.clu_data['CF_Clu_Vanz']
                data['CF_Clu_RheostatLevel'] = self.clu_data['CF_Clu_RheostatLevel'] 
                data['CF_Clu_VanzDecimal'] = self.clu_data['CF_Clu_VanzDecimal'] 
                cnt = cnt + 1
                if cnt >= 3000 :
                    cnt = 0

                #Mode Change Test
                if self.mode != self.prev_mode :
                     data['CF_Clu_CruiseSwMain'] = 1
                     self.prev_mode = self.mode
                else:
                     data['CF_Clu_CruiseSwMain'] = 0                  

                #Vel Change Test
                if self.main_acc == 1 and self.mode == 1 and self.acc_mode == False:
                    data['CF_Clu_CruiseSwState'] = 2 if data['CF_Clu_CruiseSwState'] == 0 else 0
                elif self.main_acc == 1 and self.mode == 1 and self.acc_mode == True:
                    print(self.set_dis, self.vel)
                    data['CF_Clu_CruiseSwState'] = int(self.set_sw_state(self.set_dis, self.vel)) if cnt % 5 == 0 else 0
                else:
                    data['CF_Clu_CruiseSwState'] = 0


                msg = self.db.encode_message('CLU11', data)
                can_msg = can.Message(arbitration_id=0x4f1, data=msg, is_extended_id=False)
                self.SCC.send(can_msg)
            
            except KeyboardInterrupt:
                exit(0)
            except:
                print("CLU Exception")
            time.sleep(0.02)

if __name__ == '__main__':
    TOR = TOR_Record()
    TOR.record()
    rospy.spin()

