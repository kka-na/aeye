import can
import cantools
import threading

class CanReceiver:
    def __init__(self):
        self.db = cantools.database.load_file('/home/lattepanda/Documents/can/hyundai_can.dbc')
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can1', bitrate=500000)
        self.torque = [0,0]

    def receiver(self):
        while 1:
            data = self.bus.recv(0.01)
            if (data.arbitration_id == 897):
                mdps = self.db.decode_message(data.arbitration_id, data.data)
                torque = mdps['CR_Mdps_DrvTq']
                self.torque[0] = max(self.torque[0], torque)
                self.torque[1] = min(self.torque[1], torque)
                print(self.torque)

    def resetter(self):
        while 1:
            tmp = input('Something')
            self.torque = [0,0]

if __name__ == "__main__":
    CR = CanReceiver()
    T1 = threading.Thread(target=CR.receiver)
    T2 = threading.Thread(target=CR.resetter)

    T2.start()
    T1.start()
