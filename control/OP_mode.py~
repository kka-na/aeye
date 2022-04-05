#sudo ip link set can0 up type can bitrate 500000
import can                     
import cantools                
import threading
import time
import binascii                
import os                      

class OP():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitreate=500000)
        self.db = cantools.database.load_file('/home/dreamingweaver/Downloads/hyundai_can.dbc')

        self.radar = {'main' : 0, 'speed' : 0}
        self.acc = False
        self.button = None
        self.spam = {'main' : time.time(), 'accel' : time.time(), 'acc' : time.time()}

    def receive(self):
        data = self.bus.recv()

        if data == None:
            pass
        elif (data.arbitration_id == 0x420): #SCC
            data = self.db.decode_message(data.arbitration_id, data.data)
            self.radar['main'] = data['MainMode_ACC']
            self.radar['speed'] = data['VSetDis']
        elif (data.arbitration_id == 1057): #SCC12
            data = self.db.decode_message(data.arbitration_id, data.data)
            self.acc = True if data['ACCMode'] == "enabled" else False 
        elif (data.arbitration_id == 1265): #SCC12
            data = self.db.decode_message(data.arbitration_id, data.data)
            


    def send(self, button):
        data = {'CF_Clu_CruiseSwState': 0.0, 'CF_Clu_CruiseSwMain': 0.0, 'CF_Clu_SldMainSW': 0.0, 'CF_Clu_ParityBit1': 0.0, 'CF_Clu_VanzDecimal': 0.0, 'CF_Clu_Vanz': 0.0, 'CF_Clu_SPEED_UNIT': 0.0, 'CF_Clu_DetentOut': 1.0, 'CF_Clu_RheostatLevel': 12.0, 'CF_Clu_CluInfo': 0.0, 'CF_Clu_AmpInfo': 0.0, 'CF_Clu_AliveCnt1': 0.0}

        if(self.radar['main'] == 0 and time.time()-self.spam['main'] > 0.1):
            self.spam['main'] = time.time()
            data['CF_Clu_CruiseSwMain'] = 1
            data['CF_Clu_CruiseSwState'] = 0
            msg = self.db.encode_message('CLU11', data)
            can_msg = can.Message(arbitration_id=0x4f1, data=msg, is_extended_id=False)
            self.bus.send(can_msg)

        if(self.radar['main'] == 1 and self.acc == False and time.time()-self.spam['acc'] > 0.1):
            self.spam['acc'] = time.time()
            data['CF_Clu_CruiseSwState'] = 2
            data['CF_Clu_Vanz'] = self.radar['speed']
            msg = self.db.encode_message('CLU11', data)
            can_msg = can.Message(arbitration_id=0x4f1, data=msg, is_extended_id=False)
            self.bus.send(can_msg)
        elif(self.radar['main'] == 1 and time.time()-self.spam['accel'] > 0.1 and self.acc == True):
            target = 50
            self.spam['accel'] = time.time()
            data['CF_Clu_CruiseSwState'] = self.target(self.radar['speed'], target)
            data['CF_Clu_CruiseSwMain'] = 0
            data['CF_Clu_Vanz'] = self.radar['speed']
            msg = self.db.encode_message('CLU11', data)
            can_msg = can.Message(arbitration_id=0x4f1, data=msg, is_extended_id=False)
            if(self.radar['speed'] != target):
                self.bus.send(can_msg)

    def target(self, current, target):
        if current < target:
            return 1
        elif current > target:
            return 2
        elif current == target:
            return 0

    def daemon(self):
        while 1:
            self.receive()
            self.send(0)

if __name__ == '__main__':
    OP = OP()
    OP.daemon()
    
