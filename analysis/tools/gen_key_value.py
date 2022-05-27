# Create a key-value pair with DBC
import cantools
import pprint

if __name__ == '__main__':
    CAN = dict()
    db = cantools.database.load_file('../data/hyundai_can.dbc')
    for msg in db.messages:
        # CAN[msg.frame_id] = msg.name
        CAN[msg.name] = msg.frame_id

    pprint.pprint(CAN)

