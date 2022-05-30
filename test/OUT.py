import can
import cantools
import cankey
import time
import os
import pprint

class Bridge:
    def __init__(self):
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
        self.OUT = can.Bus(
                interface='kvaser', channel=2, bitrate=500000)
        self.IN = can.Bus(
                interface='kvaser', channel=3, bitrate=500000)

        #self.OUT.set_filters(filters)
        self.SCC.set_filters(SCC_filters)
        self.cruise_on = False
        self.cnt = 0

    def bridge(self):
        while 1:
            try:
                SCC_data = self.IN.recv()
                self.OUT.send(SCC_data)
            except KeyboardInterrupt:
                exit()
            except:
                print("Exception")

if __name__ == '__main__':
    Bridge = Bridge()
    Bridge.bridge()

