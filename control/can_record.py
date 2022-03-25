import can
import cantools
import signal
import threading

import rospy
import os
import sys
from std_msgs.msg import Int8MultiArray, Int8, Int16MultiArray


'''
897 MDPS11
CR_Mdps_DrvTq -> switch & value
881 E_EMP11
SG_Brake_Pedal_Pos -> switch & value
SG_Accel_Pedal_Pos -> switch & value
688 SAS11
SG_SAS_Angle -> value
'''


class CanReceiver:
    def __init__(self):
        self.can_initialize()
        self.ros_initialize()
        self.mode = None

    def can_initialize(self):
        self.db = cantools.database.load_file(
            '/home/lattepanda/Documents/can/hyundai_can.dbc')
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can1', bitrate=500000)

        self.drvtq = 0
        self.brake_pedal = 0
        self.accel_pedal = 0
        self.sas_angle = 0

        self.can_print = True

    def ros_initialize(self):
        rospy.init_node('can_record', anonymous=True)
        rospy.Subscriber("/mode", Int8, self.mode_callback)
        self.can_record = rospy.Publisher(
            '/can_record', Int16MultiArray, queue_size=1)
        self.can_switch = rospy.Publisher(
            '/can_switch', Int8MultiArray, queue_size=1)
        self.can_record_data = Int16MultiArray()
        self.can_record_data = [0, 0, 0]
        self.can_switch_data = Int8MultiArray()
        self.can_switch_data = [0, 0, 0]
        '''
        record:
        [brake_pedal, accel_pedal, sas_angle]
        switch:
        [drvtq, brake_pedal, accel_pedal]
        '''
        self.ros_print = True

    def signal_handler(self, sig, frame):
        sys.exit(0)

    def receiver(self):
        while True:
            try:
                signal.signal(signal.SIGINT, self.signal_handler)
                msg = self.bus.recv(0.01)
                if (msg.arbitration_id == 897):
                    mdps = self.db.decode_message(msg.arbitration_id, msg.data)
                    self.drvtq = mdps['CR_Mdps_DrvTq']
                    if self.can_print:
                        print(self.drvtq)

                if (msg.arbitration_id == 881):
                    eemp = self.db.decode_message(msg.arbitration_id, msg.data)
                    self.brake_pedal = eemp['SG_Brake_Pedal_Pos']
                    self.accel_pedal = eemp['SG_Accel_Pedal_Pos']
                    if self.can_print:
                        print("Brake Pedal Pos : {}".format(self.brake_pedal))
                        print("Accel Pedal Pos : {}".format(self.accel_pedal))

                if (msg.arbitration_id == 688):
                    sas = self.db.decode_message(msg.arbitration_id, msg.data)
                    self.sas_angle = sas['SG_SAS_Angle']
                    if self.can_print:
                        print("SAS Angle : ".format(self.sas_angle))

            except Exception as e:
                exit()

    def publisher(self):
        while not rospy.is_shutdown():
            try:
                signal.signal(signal.SIGINT, self.signal_handler)
                # publish data
                record, switch = self.calculate_can()
                self.can_record_data = record
                self.can_switch_data = switch
                self.can_record.publish(self.can_record_data)
                self.can_switch.publish(self.can_switch_data)

                if self.ros_print:
                    print("CAN RECORD")
                    print(self.can_record_data)
                    print("CAN SWITCH")
                    print(self.can_switch_data)

                rospy.sleep(0.5)

            except Exception as e:
                exit()

    def calculate_can(self):
        record = self.can_record_data
        switch = [0, 0, 0]
        '''
        record:
        [brake_pedal, accel_pedal, sas_angle]
        switch:
        [drvtq, brake_pedal, accel_pedal]
        '''
        record[0] = self.brake_pedal
        record[1] = self.accel_pedal
        record[2] = self.sas_angle

        if abs(self.drvtq) > 150 and self.mode == 1:
            switch[0] = 1
        elif self.brake_pedal > 20 and self.mode == 1:
            switch[1] = 1
        elif self.accel_pedal > 30 and self.mode == 1:
            switch[2] = 1

        return record, switch

    def mode_callback(self, msg):
        self.mode = msg.data


if __name__ == "__main__":
    CR = CanReceiver()
    T1 = threading.Thread(target=CR.receiver)
    T2 = threading.Thread(target=CR.publisher)

    T2.start()
    T1.start()
