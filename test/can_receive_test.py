import can
import cantools
import cankey

db = cantools.database.load_file(
                '/home/aeye/Documents/can/hyundai_can.dbc')
can = can.Bus(interface='kvaser', channel=3, bitrate=500000)

changed = set()

while 1:
    data = can.recv()
    id = data.arbitration_id
    changed.add(id)
    print(changed)