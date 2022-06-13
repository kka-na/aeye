# Monitor the change of signals for messages SCC11 to SCC14
import can
import cantools
import copy
import os
import time

class Bridge:
    def __init__(self):
        self.db = cantools.database.load_file(
                '/home/aeye/Documents/can/hyundai_can.dbc')
        self.CCAN = can.ThreadSafeBus(
                interface='kvaser', channel='0', bitrate=500000)
        self.SCC = can.ThreadSafeBus(
                interface='kvaser', channel='1', bitrate=500000)

        # self.scc_keys = {
        #         'SCC11' : 1056,
        #         'SCC12' : 1057,
        #         'SCC13' : 1290,
        #         'SCC14' : 905
        #         }
        
        # self.yebi = {
            # 'CLU11' : 1265
            # 'CLU12' : 1456
            # 'SAS11' : 688 
            # 'TCS13' : 916 
            # 'EMS19' : 1170 
            # 'EPB11' : 1168 
            # 'WHL_SPD11' : 902 
            # 'EMS16' : 608 
            # 'TCS11' : 339 
            # 'EMS15' : 1351 
            # 'CGW1' : 1345
            # 'EMS12' : 809
            # 'ESP12' : 544 
            # 'EMS11' : 790 
            # 'TCU13' : 275 
            # 'TCU12' : 274 
            # 'TCU11' : 273 
            # 'CLU13' : 1292 
            # 'TCS15' : 1287 
            # 'P_STS' : 1136
        #     }
        self.ccan_to_scc_keys = {
            #'881' : 'E_EMS11'
            #'1173' : 'YRS13',
            # '1312' : 'CGW3',
            #'1363' : 'CGW2'
            # '1369' : 'CGW4'
            # '593' : 'MDPS12'
            # '1155' : 'FCA12'
            # '1157' : 'LFAHDA_MFC'
            # '1301' : 'CLU14'
            # '832' : 'LKAS11'
            # '1265' : 'CLU11'
            # '1456' : 'CLU12'
            # '688' : 'SAS11'
            '916' : 'TCS13'
            # '1170' : 'EMS19'
            # '1168' : 'EPB11'
            # '902' : 'WHL_SPD11'
            # '608' : 'EMS16'
            # '339' : 'TCS11'
            # '1351' : 'EMS15'
            # '1345' : 'CGW1'
            # '809' : 'EMS12'
            # '544' : 'ESP12'
            # '790' : 'EMS11'
            # '275' : 'TCU13'
            # '274' : 'TCU12'
            # '273' : 'TCU11'
            # '1292' : 'CLU13'
            # '1287' : 'TCS15'
            # '1136' : 'P_STS'
        }

        self.scc_keys = {
                '1056' : 'SCC11'
                # '1057' : 'SCC12'
                # '1290' : 'SCC13'
                # '905' : 'SCC14'
                # '909' : 'FCA11'
                # '1186' : 'FRT_RADAR11'
                # '128' : 'DCT11'
                }

        self.scc_data = {
                'SCC11' : [0 for _ in range(15)],#
                'SCC12' : [0 for _ in range(21)],#
                'SCC13' : [0 for _ in range(4)],
                'SCC14' : [0 for _ in range(6)],
                'FCA11' : [0 for _ in range(20)],#
                'FRT_RADAR11' : [0 for _ in range(1)],
                'DCT11' : [0 for _ in range(20)]
            }
        self.ccan_data = {
                'E_EMS11' : [0 for _ in range(7)],
                'YRS13' : [0 for _ in range(1)],
                'CGW3' : [0 for _ in range(4)],
                'CGW2' : [0 for _ in range(41)], 
                'CGW4' : [0 for _ in range(23)], 
                'MDPS12' : [0 for _ in range(11)], 
                'FCA12' : [0 for _ in range(2)],
                'LFAHDA_MFC' : [0 for _ in range(10)],
                'CLU14' : [0 for _ in range(26)],
                'LKAS11' : [0 for _ in range(22)],
                'CLU11' : [0 for _ in range(12)],
                'CLU12' : [0 for _ in range(1)],
                'SAS11' : [0 for _ in range(5)], 
                'TCS13' : [0 for _ in range(25)], 
                'EMS19' : [0 for _ in range(13)], 
                'EPB11' : [0 for _ in range(14)], 
                'WHL_SPD11' : [0 for _ in range(8)], 
                'EMS16' : [0 for _ in range(15)], 
                'TCS11' : [0 for _ in range(29)], 
                'EMS15' : [0 for _ in range(12)], 
                'CGW1' : [0 for _ in range(43)],
                'EMS12' : [0 for _ in range(19)],
                'ESP12' : [0 for _ in range(14)], 
                'EMS11' : [0 for _ in range(7)], 
                'TCU13' : [0 for _ in range(18)], 
                'TCU12' : [0 for _ in range(12)], 
                'TCU11' : [0 for _ in range(13)], 
                'CLU13' : [0 for _ in range(17)], 
                'TCS15' : [0 for _ in range(11)], 
                'P_STS' : [0 for _ in range(4)]
        }
        self.changed = set()
        self.rate = time.time()

    def bridge(self):
        while 1:
            # os.system('clear')
            CCAN_data = self.CCAN.recv()
            # print(CCAN_data)
            CCAN_id = CCAN_data.arbitration_id
            #print(SCC_id)

            # if CCAN_id == 1056:
            #     print(1/(time.time() - self.rate))
            #     self.rate = time.time()

            if str(CCAN_id) in self.ccan_to_scc_keys.keys():
                CCAN_parsed = self.db.decode_message(CCAN_data.arbitration_id, CCAN_data.data)
                # print(CCAN_id,CCAN_parsed)
                _id = self.ccan_to_scc_keys[str(CCAN_id)]

                for i, values in enumerate(CCAN_parsed.values()): 
                    if self.ccan_data[_id][i] != values:
                        self.changed.add(i)
                        # print(self.changed)
                        key_list = list(CCAN_parsed.keys())
                        if key_list[i] == str('ACCEnable'):
                            print(_id, key_list[i], values)

                self.ccan_data[_id] = list(CCAN_parsed.values())

    #             # self.CCAN.send(SCC_data)
                
                
    #         # CCAN_data = self.CCAN.recv()

    # def bridge(self):
    #     while 1:
    #         # os.system('clear')
    #         SCC_data = self.SCC.recv()
    #         SCC_id = SCC_data.arbitration_id
    #         # print(SCC_id)

    #         # if SCC_id == 1056:
    #         #     print(1/(time.time() - self.rate))
    #         #     self.rate = time.time()

    #         if str(SCC_id) in self.scc_keys.keys():
    #             SCC_parsed = self.db.decode_message(SCC_data.arbitration_id, SCC_data.data)
    #             # print(SCC_id,SCC_parsed)
    #             _id = self.scc_keys[str(SCC_id)]

    #             for i, values in enumerate(SCC_parsed.values()): 
    #                 if self.scc_data[_id][i] != values:
    #                     self.changed.add(i)
    #                     # print(self.changed)
    #                     key_list = list(SCC_parsed.keys())
    #                     print(_id, key_list[i], values)

    #             self.scc_data[_id] = list(SCC_parsed.values())

if __name__ == '__main__':
    Bridge = Bridge()
    Bridge.bridge()
    

