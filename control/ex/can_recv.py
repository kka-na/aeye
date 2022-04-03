import can
import cantools
import threading
import os
import time

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

    def can_initialize(self):
        self.db = cantools.database.load_file(
            '/home/lattepanda/Documents/can/hyundai_can.dbc')
        # self.bus0 = can.ThreadSafeBus(
        #     interface='socketcan', channel='can0', bitrate=500000)
        self.bus1 = can.ThreadSafeBus(
            interface='socketcan', channel='can1', bitrate=500000)

        self.drvtq = 0

        self.customleft = None
        self.customright = 2


    def receiver(self):
        while True:
            msg = self.bus1.recv(0.01)
            if (msg.arbitration_id == 897):
                mdps = self.db.decode_message(msg.arbitration_id, msg.data)
                self.drvtq = mdps['CR_Mdps_DrvTq']

            # if (msg.arbitration_id == 881):
            #     eemp = self.db.decode_message(msg.arbitration_id, msg.data)
            #     self.brake_pedal = eemp['Brake_Pedal_Pos']
            #     self.accel_pedal = eemp['Accel_Pedal_Pos']
            #     print("Brake Pedal Pos : {}".format(self.brake_pedal))
            #     print("Accel Pedal Pos : {}".format(self.accel_pedal))

            # if (msg.arbitration_id == 688):
            #     sas = self.db.decode_message(msg.arbitration_id, msg.data)
            #     self.sas_angle = sas['SAS_Angle']
            #     print("SAS Angle : ".format(self.sas_angle))

            if (msg.arbitration_id == 1060): 
                lka = self.db.decode_message(msg.arbitration_id, msg.data)
                self.customleft = lka['custom_bsd_left']
                self.customright = lka['custom_bsd_right']
            # time.sleep(1)


        #cantools
        # 
    
    def pprint(self):
        while 1:
            print("Toq : {}".format(self.drvtq))
            print("customLeft : {} , customRight : {}".format(self.customleft, self.customright))
            os.system('clear')
            time.sleep(0.1)


if __name__ == "__main__":
    CR = CanReceiver()
    T1 = threading.Thread(target=CR.receiver)
    T2 = threading.Thread(target=CR.pprint)

    T1.start()
    T2.start()
