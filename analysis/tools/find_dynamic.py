# Find static and dynamic messages
import cantools
import cankey
import pprint
from collections import defaultdict

def get_hst():
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')
    data.readline()
    return data

def get(db, data):
    _flag = cankey.DATA.copy() 
    _past = cankey.DATA.copy() 
    _changes = defaultdict(set)

    for lines in data:
        if len(lines) == 17:
            break
        parsed = lines.split()
        id, dlc = parsed[1], int(parsed[2])
        frame = list(map(int,parsed[3:3+dlc]))
        if id in cankey.DATA and id not in ('1191'):
            _current = db.decode_message(int(id), frame)
            for key in _current:
                if _current[key] != _past[id][key] and _flag[id][key] == True:
                    ids = cankey.ID[int(id)]
                    _changes[ids].add(key)
                elif _current[key] != _past[id][key]:
                    _flag[id][key] = True
        _past[id] = _current

    pprint.pprint(_changes)
                
if __name__ == '__main__':
    db = cantools.database.load_file('../data/hyundai_can.dbc')
    data = get_hst()
    get(db, data)



