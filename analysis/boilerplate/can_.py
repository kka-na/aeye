import can
import cantools
import time

if __name__ == '__main__':
    CCAN = can.ThreadSafeBus(interface='socketcan', channel='can0', bitreate=500000)
    SCC = can.ThreadSafeBus(interface='socketcan', channel='can1', bitreate=500000)

    SCC_filter = [
            {"can_id":1056, 'can_mask' : 0xfff, 'extended': False},
            {"can_id":1057, 'can_mask' : 0xfff, 'extended': False},
            {"can_id":1290, 'can_mask' : 0xfff, 'extended': False},
            {"can_id":909, 'can_mask' : 0xfff, 'extended': False},
            {"can_id":905, 'can_mask' : 0xfff, 'extended': False},
            # {"can_id":1157, 'can_mask' : 0xfff, 'extended': False},
            {"can_id":1186, 'can_mask' : 0xfff, 'extended': False}
            # {"can_id":128, 'can_mask' : 0xfff, 'extended': False},
            ]
    SCC.set_filters(SCC_filter)
    alpha = time.time()

    while 1:
        msg = SCC.recv()
        print(time.time() - alpha)
        alpha = time.time()
        # print(msg)
        CCAN.send(msg)
    

    
