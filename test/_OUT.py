import can
import cantools
import cankey
import time
import os
import pprint

class Bridge:
    def __init__(self):
        self.CCAN = can.Bus(
                interface='kvaser', channel=0, bitrate=500000)
        self.SCC = can.Bus(
                interface='kvaser', channel=1, bitrate=500000)

        self.CCAN.set_filters(cankey.CCAN_filters)
        self.SCC.set_filters(cankey.SCC_filters)
        self.cruise_on = False
        self.cnt = 0

    def bridge(self):
        while 1:
            try:
                SCC_data = self.CCAN.recv()
                CCAN_data = self.SCC.recv(0.015)

                self.SCC.send(SCC_data)
                self.CCAN.send(CCAN_data)

            except KeyboardInterrupt:
                exit()
            except Exception as e:
                print("Exception")
                print(e)

if __name__ == '__main__':
    Bridge = Bridge()
    Bridge.bridge()

