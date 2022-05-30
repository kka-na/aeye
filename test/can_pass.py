import can
import cantools
import cankey
import time
import os

class Bridge:
    def __init__(self):
        self.db = cantools.database.load_file(
                '/home/aeye/Documents/can/hyundai_can.dbc')
        filters = [
                {"can_id": 1265,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1456,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 688,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 916,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1168,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 902,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 339,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1345,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 544,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1292,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 1287,  "can_mask" : 0xfff, "extended": False},
                {"can_id": 832,  "can_mask" : 0xfff, "extended": False},
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
        self.CCAN_pass = can.RedirectReader(self.CCAN)
        self.SCC_pass = can.RedirectReader(self.SCC)
            

        # self.CCAN.set_filters(filters)
        # self.SCC.set_filters(SCC_filters)
        self.cruise_on = False
        self.cnt = 0

    def scc_escape(self, SCC_data, SCC_id):
        if SCC_id == cankey.key['SCC11']:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            data['Navi_SCC_Camera_Status'] = 2
            if self.cruise_on:
                data['MainMode_ACC'] = 1
            else:
                data['MainMode_ACC'] = 0 
              
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        elif SCC_id == cankey.key['FCA11']:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            data['FCA_Failinfo'] = 0
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        else:
            data = self.db.decode_message(SCC_id, SCC_data.data)
            data = self.db.encode_message(SCC_id, data)
            data = can.Message(arbitration_id=SCC_id, data=data, is_extended_id=False)
            return data
        #return SCC_data
        
    def ccan_escape(self, CCAN_data, CCAN_id):
        if CCAN_id == cankey.ckey['TCS13']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            data['ACCEnable'] = 1
            data['CF_VSM_Avail'] = 0
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['LKAS11']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            data['CF_Lkas_LdwsSysState'] = 0
            data['CF_Lkas_FusionState'] = 0
            data['CF_Lkas_FcwOpt_USM'] = 0
            data['CF_Lkas_LdwsOpt_USM'] = 2
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['TCS15']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            data['AVH_LAMP'] = 2
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['CGW2']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            data['CF_Gway_SJBDeliveryMode'] = 0
            data['CF_Gway_WiperParkPosition'] = 0
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        elif CCAN_id == cankey.ckey['CLU11']:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            cruise_sw = int(data['CF_Clu_CruiseSwMain'])
            if cruise_sw and not self.cruise_on:
                self.cruise_on = True
                print('TRUE')
            elif cruise_sw and self.cruise_on:
                self.cruise_on = False
                print('False')
            self.cnt += 1

            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
            # data = self.db.encode_message(CCAN_id, data)
            # data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            # return data
        else:
            data = self.db.decode_message(CCAN_id, CCAN_data.data)
            data = self.db.encode_message(CCAN_id, data)
            data = can.Message(arbitration_id=CCAN_id, data=data, is_extended_id=False)
            return data
        #return CCAN_data

    def timekeeper(self, SCC_id):
        SCC_id = str(SCC_id)
        hz = 1/(time.time() - cankey.past[SCC_id])
        cankey.time[SCC_id] = (hz + cankey.time[SCC_id])/2
        cankey.past[SCC_id] = time.time()

    def bridge(self):
        while 1:
            start = time.time()
            SCC_data = self.SCC.recv()
            
            # SCC_id = SCC_data.arbitration_id
            # SCC_data = self.scc_escape(SCC_data, SCC_id)
            # # self.timekeeper(SCC_id)
            # self.CCAN.send(SCC_data)

            CCAN_data = self.CCAN.recv()
            self.CCAN_pass.on_message_received(SCC_data)
            self.SCC_pass.on_message_received(CCAN_data)

            print(1/(time.time() - start))

            
            # CCAN_id = CCAN_data.arbitration_id
            # CCAN_data = self.ccan_escape(CCAN_data, CCAN_id)
            # self.SCC.send(CCAN_data)
        #    if CCAN_data.arbitration_id == cankey.ckey['CLU11'] or CCAN_data.arbitration_id == cankey.ckey['TCS13']:

if __name__ == '__main__':
    Bridge = Bridge()
    Bridge.bridge()

