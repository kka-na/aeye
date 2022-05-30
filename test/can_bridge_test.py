import can
import cankey
import time
import os
import threading

class Bridge:
    def __init__(self):
        self.CCAN = can.ThreadSafeBus(
                interface='kvaser', channel=0, bitrate=500000)
        self.SCC = can.ThreadSafeBus(
                interface='kvaser', channel=1, bitrate=500000)

        self.CCAN.set_filters(cankey.CCAN_filters)
        self.SCC.set_filters(cankey.SCC_filters)

    def bridge1(self):
        while 1:
            SCC_data = self.SCC.recv()
            self.CCAN.send(SCC_data)
            
    def birdge2(self):
        while 1:
            CCAN_data = self.CCAN.recv()
            self.SCC.send(CCAN_data)

if __name__ == '__main__':
    Bridge = Bridge()
    th1 = threading.Thread(target=Bridge.bridge1)
    th2 = threading.Thread(target=Bridge.birdge2)

    th1.start()
    th2.start()

