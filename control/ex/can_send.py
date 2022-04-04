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
        self.bus0 = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        # self.bus1 = can.ThreadSafeBus(
            # interface='socketcan', channel='can1', bitrate=500000)

        self.bsd = {'custom_bsd_left':0, 'custom_bsd_right':1}


    def sender(self):
        while True:
            bsd_msg = self.db.encode_message('aeye',self.bsd)
            frame = can.Message(arbitration_id=1060, data=bsd_msg)

            self.bus0.send(frame)

            time.sleep(0.1)


        #cantools
        # 
    


if __name__ == "__main__":
    CR = CanReceiver()
    T1 = threading.Thread(target=CR.sender)

    T1.start()
