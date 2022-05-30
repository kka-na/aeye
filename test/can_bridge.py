import can
import cantools
import cankey
import time
import os
import pprint

class Bridge:
    def __init__(self):
        self.counter = 0
        self.db = cantools.database.load_file(
                '/home/aeye/Documents/can/hyundai_can.dbc')
        filters = [
                {"can_id": 1265,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1456,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 688,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 916,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1268,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1168,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 902,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 339,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1345,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 544,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1292,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1287,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 832,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1157,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 593,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1363,  "can_mask" : 0xfff, "extended": False}
                ]
        SCC_filters = [
                {"can_id": 1056,  "can_mask" : 0xfff, "extended": False},
                ]
        self.CCAN = can.Bus(
                interface='kvaser', channel=0, bitrate=500000)
        self.SCC = can.Bus(
                interface='kvaser', channel=1, bitrate=500000)
        self.OUT = can.Bus(
                interface='kvaser', channel=2, bitrate=500000)
        self.IN = can.Bus(
                interface='kvaser', channel=3, bitrate=500000)

        self.CCAN.set_filters(filters)
        # self.SCC.set_filters(SCC_filters)
        self.cruise_on = False
        self.cnt = 0

    def scc_escape(self, SCC_data, SCC_id):
        if SCC_id == cankey.key['SCC11']:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            # data['Navi_SCC_Camera_Status'] = 2
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        elif SCC_id == cankey.key['SCC12']:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            # data['ACCFailInfo'] = 0
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        elif SCC_id == cankey.key['FCA11']:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            # data['FCA_Failinfo'] = 0
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        elif SCC_id == cankey.key['LFAHDA_MFC']:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        else:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        
    def ccan_escape(self, CCAN_data, CCAN_id):
        CCAN_id = int(CCAN_id)
        if CCAN_id == cankey.ckey['CLU11']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            # data['ACCEnable'] = 1
            # data['CF_VSM_Avail'] = 0
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['LKAS11']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            # data['CF_Lkas_LdwsSysState'] = 0
            # data['CF_Lkas_FusionState'] = 0
            # data['CF_Lkas_FcwOpt_USM'] = 0
            # data['CF_Lkas_LdwsOpt_USM'] = 2
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['SPAS11']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            # data['AVH_LAMP'] = 2
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['SPAS12']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            # data['CF_Gway_SJBDeliveryMode'] = 0
            # data['CF_Gway_WiperParkPosition'] = 0
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['LFAHDA_MFC']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            # data['HDA_USM'] = 0
            # data['LFA_USM'] = 0
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        else:
            data = can.Message(arbitration_id=CCAN_id, data=CCAN_data.data, is_extended_id=False)
            return data

    def timekeeper(self, SCC_id):
        SCC_id = str(SCC_id)
        hz = 1/(time.time() - cankey.past[SCC_id])
        cankey.time[SCC_id] = (hz + cankey.time[SCC_id])/2
        cankey.past[SCC_id] = time.time()

    def unpackpack(self, _data, _id):
        data = self.db.decode_message(_id, _data.data)
        data = self.db.encode_message(_id, data)
        data = can.Message(arbitration_id=_id, data=data, is_extended_id=False)
        return data 
         
    def bridge(self):
        while 1:
            # try:
            # start = time.time()
            SCC_data = self.SCC.recv()
            self.CCAN.send(SCC_data)
            # SCC_data.channel = 0
            # SCC_data = can.Message(arbitration_id=SCC_id, data=SCC_data.data, is_extended_id=False)
            # SCC_data = self.scc_escape(SCC_data, SCC_id)
            # self.timekeeper(SCC_id)
            # self.unpackpack(SCC_data, SCC_id)
            
            # CCAN_data = self.IN.recv()
            # CCAN_data.channel = 1
            # CCAN_data = self.ccan_escape(CCAN_data, CCAN_id)
            # # self.unpackpack(CCAN_data, CCAN_id)
            # self.OUT.send(CCAN_data)
            # except KeyboardInterrupt:
                # exit(0)
            # except Exception as e:
                # print("Exception")
                # print(e)

if __name__ == '__main__':
    Bridge = Bridge()
    Bridge.bridge()

