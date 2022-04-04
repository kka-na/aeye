import rospy
import can
import cantools
import os
# import threading
from std_msgs.msg import String, Float32, Int8
from std_msgs.msg import Int8MultiArray, Int16MultiArray
import time
import sys
import signal

'''
897 MDPS11
CR_Mdps_DrvTq -> switch & value
881 E_EMP11
SG_Brake_Pedal_Pos -> switch & value
SG_Accel_Pedal_Pos -> switch & value
688 SAS11
SG_SAS_Angle -> value
'''

class Activate_Signal_Interrupt_Handler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('\nYou pressed Ctrl+C! Never use Ctrl+Z!')
        sys.exit(0)

class CanReceiver:
    def __init__(self):        
        self.can_initialize()
        self.ros_initialize()
        self.mode = None

    def can_initialize(self):
        self.db = cantools.database.load_file(
            '/home/lattepanda/Documents/can/hyundai_can.dbc')
        try:
            self.bus = can.interface.Bus(
            interface='socketcan', channel='can0', bitrate=500000)
        except OSError:
            print('Exit the program. Check the system, please.')
            exit()
        
        self.WHEELSIZE = 0.432 #m
        
        self.drvtq = 0
        
        self.brake_pedal = 0
        self.accel_pedal = 0
        
        self.sas_angle = 0
        self.rcv_wheel_speed  = 0
        self.rpm = 0

        self.can_print = True

    def ros_initialize(self):
        rospy.init_node('can_reciever', anonymous=True)

        rospy.Subscriber("/mode", Int8, self.mode_callback)
        rospy.Subscriber("/estop", Int8, self.mode_callback)

        self.can_record = rospy.Publisher('/can_record', Int16MultiArray, queue_size=1)
        self.can_switch = rospy.Publisher( '/can_switch', Int8MultiArray, queue_size=1)
        self.mode_pub = rospy.Publisher('/mode_set', Int8, queue_size=1)

        self.can_record_data = Int16MultiArray()
        self.can_record_data.data = [0, 0, 0, 0, 0, 0]

        self.can_switch_data = Int8MultiArray()
        self.can_switch_data.data = [0, 0, 0]


        #Raw data
        self.rcv_gear = 'P'
        # self.rcv_wheel_speed = 1.0 #km/h
        # self.rcv_steer_angle = None
        # self.rcv_steer_tq = None

        # self.res = {'CF_Clu_CruiseSwState': 0.0, 'CF_Clu_CruiseSwMain': 0.0, 'CF_Clu_SldMainSW': 0.0, 'CF_Clu_ParityBit1': 0.0, 'CF_Clu_VanzDecimal': 0.0, 'CF_Clu_Vanz': 0.0, 'CF_Clu_SPEED_UNIT': 0.0, 'CF_Clu_DetentOut': 0.0, 'CF_Clu_RheostatLevel': 12.0, 'CF_Clu_CluInfo': 0.0, 'CF_Clu_AmpInfo': 0.0, 'CF_Clu_AliveCnt1': 9.0} 

        self.gear_map = {'P' : 0,'R' : 1, 'N' : 2, 'D' : 3}
        
        self.ros_print = False

        '''
        record:
        [brake_pedal, accel_pedal, sas_angle, gear, rpm, vel]
        switch:
        [drvtq, brake_pedal, accel_pedal]
        '''

    def receiver(self):
        msg = self.bus.recv(0.01)
 #       print(msg)
        try:
            if (msg.arbitration_id == 897):
                mdps = self.db.decode_message(msg.arbitration_id, msg.data)
                self.drvtq = mdps['CR_Mdps_DrvTq']
                if self.can_print:
                    print("Drive Torque : {}".format(self.drvtq))

            if (msg.arbitration_id == 881):
                eemp = self.db.decode_message(msg.arbitration_id, msg.data)
                self.brake_pedal = eemp['Brake_Pedal_Pos']
                self.accel_pedal = eemp['Accel_Pedal_Pos']

                if self.can_print:
                    print("Brake Pedal Pos : {}".format(self.brake_pedal))
                    print("Accel Pedal Pos : {}".format(self.accel_pedal))

            if (msg.arbitration_id == 688):
                sas = self.db.decode_message(msg.arbitration_id, msg.data)
                self.sas_angle = sas['SAS_Angle']
                if self.can_print:
                    print("SAS Angle : ".format(self.sas_angle))

            #WHL_SPD11 id 902
            if (msg.arbitration_id == 0x386):
                whl_spd11 = self.db.decode_message(msg.arbitration_id, msg.data)

                self.velocity_RL = whl_spd11['WHL_SPD_RL']
                self.velocity_RR = whl_spd11['WHL_SPD_RR']
                self.rcv_wheel_speed  = (self.velocity_RR + self.velocity_RL) * 0.5
                self.rpm = self.rcv_wheel_speed / (0.1885*self.WHEELSIZE)
            
            #elecgear id 882
            if (msg.arbitration_id == 0x372):
                elecgear = self.db.decode_message(msg.arbitration_id, msg.data)
                self.rcv_gear = elecgear['Elect_Gear_Shifter']
        except Exception as e:
            print(e)

    def publisher(self):


        self.can_record.publish(self.can_record_data)
        self.can_switch.publish(self.can_switch_data)

        if self.ros_print:
            print("CAN RECORD")
            print(self.can_record_data)
            print("CAN SWITCH")
            print(self.can_switch_data)
            os.system('clear')
        

    def calculate_can(self):
        record = self.can_record_data.data

        switch = [0, 0, 0]
        '''
        record:
        [brake_pedal, accel_pedal, sas_angle, gear, rpm, vel]
        switch:
        [drvtq, brake_pedal, accel_pedal]
        '''
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

    def mode_callback(self, msg):
        self.mode = msg.data

    def estop_callback(self, msg):
        self.estop = msg.data


    def run(self):
        cur_time = time.time()
        while True:
            self.receiver()
            if time.time() - cur_time > 0.02:
                self.can_record_data.data, self.can_switch_data.data = self.calculate_can()
                print('-------------')
                self.publisher()
                cur_time = time.time()



    # def run(self):
        # while True:
            # self.msg_gear.data = self.gear_map[str(self.rcv_gear)]
            # self.msg_rpm.data = self.rcv_wheel_speed / (0.1885*self.WHEELSIZE) #km/h
            # self.msg_vel.data = int(self.rcv_wheel_speed)
            # self.msg_steer.data = self.rcv_steer_angle
            # self.pub_gear.publish(self.msg_gear)
            # self.pub_rpm.publish(self.msg_rpm)
            # self.pub_vel.publish(self.msg_vel)
            # self.pub_steer.publish(self.msg_steer)
            # rospy.sleep(0.1)


if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    CR = CanReceiver()
    CR.run()
