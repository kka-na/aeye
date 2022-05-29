import cankey
import cantools
import pprint

if __name__ == '__main__':
    db = cantools.database.load_file('../data/hyundai_can.dbc')
    while 1:
        id = int(input('Enter ID : '))
        if id in cankey.ID:
            name = cankey.ID[id]
            msg = db.decode_message(name, [0 for _ in range(cankey.DLC[id])])
            print(name)
            pprint.pprint(msg)
            print('____________________\n)')
            
        else:
            print('No such id')
