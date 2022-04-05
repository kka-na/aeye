#sudo ip link set can0 up type can bitrate 500000
import can                     
import cantools                
import threading
import time
import binascii                
import os    
import rospy 
from std_msgs.msg import Int8, Int8MultiArray

class OP():
    def __init__(self):
        rospy.init_node('op_mode', anonymous=True)
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can1', bitreate=500000)
        self.db = cantools.database.load_file('/home/lattepanda/Documents/can/hyundai_can.dbc')

        self.bsd_array = [0, 0]
        self.radar = {'main' : 0, 'speed' : 0}
        self.acc = False
        self.spam = {'main' : time.time(), 'accel' : time.time(), 'acc' : time.time(), 'bsd' : time.time()}
        self.cnt = 0 #For Speed switch ON
        self.sas_angle = 0

        self.acc_pub = rospy.Publisher("/acc", Int8, queue_size=1)
        rospy.Subscriber('/mode', Int8, self.mode_callback)
        rospy.Subscriber('/target_vel', Int8, self.speed_callback)
        rospy.Subscriber("/BSD_check", Int8MultiArray, self.BSD_callback)

        self.recv_mode = 0
        self.recv_speed = 0
    
    def mode_callback(self, msg):
        self.recv_mode = msg.data
    
    def speed_callback(self, msg):
        self.recv_speed = msg.data

    def BSD_callback(self, msg):
        self.bsd_array = msg.data

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
        elif (data.arbitration_id == 1265): #Debug
            data = self.db.decode_message(data.arbitration_id, data.data)
        elif (msg.arbitration_id == 688):
                sas = self.db.decode_message(msg.arbitration_id, msg.data)
                self.sas_angle = sas['SAS_Angle']

    def send(self, mode, target): #add mode
        data = {'CF_Clu_CruiseSwState': 0.0, 'CF_Clu_CruiseSwMain': 0.0, 'CF_Clu_SldMainSW': 0.0, 'CF_Clu_ParityBit1': 0.0, 'CF_Clu_VanzDecimal': 0.0, 'CF_Clu_Vanz': 0.0, 'CF_Clu_SPEED_UNIT': 0.0, 'CF_Clu_DetentOut': 1.0, 'CF_Clu_RheostatLevel': 12.0, 'CF_Clu_CluInfo': 0.0, 'CF_Clu_AmpInfo': 0.0, 'CF_Clu_AliveCnt1': 0.0}

        if(self.radar['main'] == 0 and mode == 1 and time.time()-self.spam['main'] > 0.1): #Main Switch On
            self.spam['main'] = time.time()
            data['CF_Clu_CruiseSwMain'] = 1 
            data['CF_Clu_CruiseSwState'] = 0
            self.sender(data)

        elif(self.radar['main'] == 1 and mode == 0 and time.time()-self.spam['main'] > 0.1): #Main Switch Off
            self.spam['main'] = time.time()
            data['CF_Clu_CruiseSwMain'] = 1 
            data['CF_Clu_CruiseSwState'] = 0
            self.sender(data)

        if(self.radar['main'] == 1 and mode == 1 and self.acc == False and time.time()-self.spam['acc'] > 0.1): #Speed Switch On
            self.spam['acc'] = time.time()
            data['CF_Clu_CruiseSwState'] = 2
            data['CF_Clu_Vanz'] = self.radar['speed']
            if(self.cnt < 20):
                self.sender(data)
                self.cnt =  0
                self.acc_pub.publish(1)
            elif(self.acc == False):
                self.acc_pub.publish(0)
                self.cnt =  0

        elif(self.radar['main'] == 1 and mode == 1 and time.time()-self.spam['accel'] > 0.1 and self.acc == True): #Goto target
            self.spam['accel'] = time.time()
            data['CF_Clu_CruiseSwState'] = self.target(self.radar['speed'], target)
            data['CF_Clu_CruiseSwMain'] = 0
            data['CF_Clu_Vanz'] = self.radar['speed']
            if(self.radar['speed'] != target):
                self.sender(data)


        if(time.time() - self.spam['bsd'] > 0.5): #BSD
            self.spam['bsd'] = time.time()
            data = {'custom_bsd_left':self.bsd_array[0], 'custom_bsd_right':self.bsd_array[1]}
            self.sender(data, clu=False)

    def target(self, current, target):
        if(self.sas_angle > 10):
            target = min(target, target - int(abs(self.sas_angle)/20))
        if current < target:
            return 1
        elif current > target:
            return 2
        elif current == target:
            return 0

    def sender(self, data, clu=True):
        if (clu == True):
            msg = self.db.encode_message('CLU11', data) #Button Data 
            can_msg = can.Message(arbitration_id=0x4f1, data=msg, is_extended_id=False)
        else:
            msg = self.db.encode_message('aeye', data) #Custom AEye data for BSD
            can_msg = can.Message(arbitration_id=1060, data=msg, is_extended_id=False)
        self.bus.send(can_msg)

    def daemon(self):
        temp_mode = 0
        temp_target = 0
        while 1:
            self.receive()
            if temp_mode != self.recv_mode:
                temp_mode = self.recv_mode
            if temp_target != self.recv_speed:
                temp_target = self.recv_speed
            self.send(temp_mode, temp_target)

if __name__ == '__main__':
    OP = OP()
    OP.daemon()
    
