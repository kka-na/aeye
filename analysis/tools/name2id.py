import cankey
import cantools
import pprint

if __name__ == '__main__':
    db = cantools.database.load_file('../data/hyundai_can.dbc')

    while 1:
        name = input('Enter name : ')
        name = name.upper()

        if name in cankey.NAME:
            id = cankey.NAME[name]
            msg = db.decode_message(name, [0 for _ in range(cankey.DLC[id])])
            print(id)
            pprint.pprint(msg)
            print('____________________\n')




        else:
            print('No such name')
