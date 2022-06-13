import can
import cantools
import cankey

import time
import os
import sys
import signal

import threading

import rospy
from std_msgs.msg import String, Float32, Int8
from std_msgs.msg import Int8MultiArray, Int16MultiArray

class Activate_Signal_Interrupt_Handler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('\nYou pressed Ctrl+C! Never use Ctrl+Z!')
        sys.exit(0)

class Bridge:
    def __init__(self):
        #######CAN############
        
        self.db = cantools.database.load_file(
                '/home/aeye/Documents/can/hyundai_can.dbc')
        self.CCAN = can.ThreadSafeBus(
                interface='kvaser', channel=0, bitrate=500000)
        self.SCC = can.ThreadSafeBus(
                interface='kvaser', channel=1, bitrate=500000)
        self.CCAN.set_filters(cankey.CCAN_filters)
        self.SCC.set_filters(cankey.SCC_filters)

        self.counter = 0
        self.WHEELSIZE = 0.432 #m
        self.drvtq = 0
        self.brake_pedal = 0
        self.accel_pedal = 0
        self.sas_angle = 0
        self.rcv_wheel_speed  = 0
        self.rpm = 0

        self.rcv_gear = 'P'
        # self.rcv_wheel_speed = 1.0 #km/h
        # self.rcv_steer_angle = None
        # self.rcv_steer_tq = None
        '''self.res = {'CF_Clu_CruiseSwState': 0.0, 
        'CF_Clu_CruiseSwMain': 0.0, 'CF_Clu_SldMainSW': 0.0,
        'CF_Clu_ParityBit1': 0.0, 'CF_Clu_VanzDecimal': 0.0, 
        'CF_Clu_Vanz': 0.0, 'CF_Clu_SPEED_UNIT': 0.0, 
        'CF_Clu_DetentOut': 0.0, 'CF_Clu_RheostatLevel': 12.0, 
        'CF_Clu_CluInfo': 0.0, 'CF_Clu_AmpInfo': 0.0, 
        'CF_Clu_AliveCnt1': 9.0} '''

        self.gear_map = {'P' : 0,'R' : 1, 'N' : 2, 'D' : 3}          


        self.mode = 0
        self.prev_mode = 0
        self.vel = 0
        self.prev_vel = 0
        self.main_acc = 0 #from SCC11  
        self.set_dis = 0 #from SCC11
        self.acc_mode = False #from SCC12
        self.sas_angle = 0 #from SAS11
        self.clu_data = {'CF_Clu_Vanz' : 0, 
                        'CF_Clu_RheostatLevel' : 0, 
                        'CF_Clu_VanzDecimal' : 0}

        ########ROS################
        rospy.init_node('Bridge_CAN', anonymous=False)
        # Mode change Test
        rospy.Subscriber('/mode', Int8, self.mode_set_callback)
        rospy.Subscriber('/target_vel', Int8, self.vel_set_callback)
        rospy.Subscriber("/estop", Int8, self.estop_callback)

        self.can_record = rospy.Publisher('/can_record', Int16MultiArray, queue_size=1)
        self.can_switch = rospy.Publisher( '/can_switch', Int8MultiArray, queue_size=1)
        self.mode_pub = rospy.Publisher('/mode_set', Int8, queue_size=1)

        self.can_record_data = Int16MultiArray()
        self.can_record_data.data = [0, 0, 0, 0, 0, 0]

        self.can_switch_data = Int8MultiArray()
        self.can_switch_data.data = [0, 0, 0]

        '''
        record:
        [brake_pedal, accel_pedal, sas_angle, gear, rpm, vel]
        switch:
        [drvtq, brake_pedal, accel_pedal]
        '''

    def mode_set_callback(self, msg):
        self.mode = msg.data

    def vel_set_callback(self, msg):
        self.vel = int(msg.data)

    def estop_callback(self, msg):
        self.estop = msg.data

    def set_scc11(self, data): #1056
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.main_acc = data['MainMode_ACC']
        self.set_dis = data['VSetDis']

    def set_scc12(self, data): #1057
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.acc_mode = True if data['ACCMode'] == "enabled" else False
    
    def set_sas11(self, data): #688
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.sas_angle = data['SAS_Angle']

    def set_clu11(self, data): #688
        data = self.db.decode_message(data.arbitration_id, data.data)
        self.clu_data['CF_Clu_Vanz'] = data['CF_Clu_Vanz']
        self.clu_data['CF_Clu_RheostatLevel'] = data['CF_Clu_RheostatLevel']
        self.clu_data['CF_Clu_VanzDecimal'] = data['CF_Clu_VanzDecimal']
        
    def bridge(self):
        while 1:
            try:
                #Send SCC to CCAN
                SCC_data = self.SCC.recv(0)
                if SCC_data != None:
                    SCC_id = SCC_data.arbitration_id
                    if SCC_id == 1056:
                        self.set_scc11(SCC_data)
                    elif SCC_id == 1057:
                        self.set_scc12(SCC_data)
                    elif SCC_id == 688:
                        self.set_sas11(SCC_data)
                    self.CCAN.send(SCC_data)

                #Send CCAN to SCC ( Except CLU11 )
                CCAN_data =self.CCAN.recv(0)                
                if CCAN_data != None:
                    if CCAN_data.arbitration_id == 1265:
                        self.set_clu11(CCAN_data)
                        continue
                    if CCAN_data.arbitration_id == 897:
                        mdps = self.db.decode_message(msg.arbitration_id, msg.data)
                        self.drvtq = mdps['CR_Mdps_DrvTq']
                        # print("Drive Torque : {}".format(self.drvtq))
                    if CCAN_data.arbitration_id == 881:
                        eemp = self.db.decode_message(msg.arbitration_id, msg.data)
                        self.brake_pedal = eemp['Brake_Pedal_Pos']
                        self.accel_pedal = eemp['Accel_Pedal_Pos']
                        # print("Brake Pedal Pos : {}".format(self.brake_pedal))
                        # print("Accel Pedal Pos : {}".format(self.accel_pedal))
                    if CCAN_data.arbitration_id == 688:
                        sas = self.db.decode_message(msg.arbitration_id, msg.data)
                        self.sas_angle = sas['SAS_Angle']
                        # print("SAS Angle : ".format(self.sas_angle))
                    #WHL_SPD11 id 902    0x386
                    if CCAN_data.arbitration_id == 902):
                        whl_spd11 = self.db.decode_message(msg.arbitration_id, msg.data)
                        self.velocity_RL = whl_spd11['WHL_SPD_RL']
                        self.velocity_RR = whl_spd11['WHL_SPD_RR']
                        self.rcv_wheel_speed  = (self.velocity_RR + self.velocity_RL) * 0.5
                        self.rpm = self.rcv_wheel_speed / (0.1885*self.WHEELSIZE)
                    #elecgear id 882  0x372
                    if CCAN_data.arbitration_id == 882):
                        elecgear = self.db.decode_message(msg.arbitration_id, msg.data)
                        self.rcv_gear = elecgear['Elect_Gear_Shifter']

                    self.SCC.send(CCAN_data)

            except KeyboardInterrupt:
                exit(0)
            except Exception as e:
                print("Exception")    
                print(e)
    
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
    Activate_Signal_Interrupt_Handler()
    
    Bridge = Bridge()
    th1 = threading.Thread(target=Bridge.bridge)
    th2 = threading.Thread(target=Bridge.send_clu)
    th1.daemon = True
    th2.daemon = True

    try:
        th1.start()
        th2.start()
    except KeyboardInterrupt as e:
        sys.exit(e)
    
    rospy.spin()

