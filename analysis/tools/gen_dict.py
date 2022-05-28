# Create a key-signal pair with DBC
import cantools
import fetch_id
import cankey
import pprint

def get_hst():
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')
    return data

if __name__ == '__main__':
    CAN = dict()
    data = get_hst()
    id_set = fetch_id.get(data)

    db = cantools.database.load_file('../data/hyundai_can.dbc')
    for id in id_set:
        id = int(id)
        try:
            struct = db.decode_message(id, [0 for _ in range(cankey.DLC[id])])
            CAN[str(id)] = struct
        except:
            pass

    pprint.pprint(CAN)

