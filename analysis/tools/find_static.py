# Find static and dynamic messages V2
import cantools
import cankey
import pprint
from collections import defaultdict

def get_hst():
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/CCAN_false_radar.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')
    data.readline()
    return data

def init_frequency(frequency):
    for keys in frequency:
        frequency[keys] = dict.fromkeys(frequency[keys], int(0))
    return frequency

def get_static(past, frequency):
    for message in frequency:
        for signal in frequency[message]:
            msg = frequency[message]
            if msg[signal] > 10:
                del past[message][signal]
    return past

def get(db, data):
    _flag = cankey.DATA.copy() 
    _past = cankey.DATA.copy() 
    _frequency = init_frequency(cankey.DATA.copy())

    for lines in data:
        if len(lines) == 17:
            break
        parsed = lines.split()
        id, dlc = parsed[1], int(parsed[2])
        frame = list(map(int,parsed[3:3+dlc]))

        if id in cankey.DATA and id not in ('1191'): #1191 DLC is different
            _current = db.decode_message(int(id), frame)
            for key in _current:
                if _current[key] != _past[id][key] and _flag[id][key] == True: #Exclude first Data
                    _frequency[id][key] = _frequency[id][key] + 1 #Add to frequency
                else:
                    _flag[id][key] = True

            _past[id] = _current

    _static = get_static(_past, _frequency)
    return _static
                
if __name__ == '__main__':
    db = cantools.database.load_file('../data/hyundai_can.dbc')
    data = get_hst()
    static = get(db, data)
    # pprint.pprint(static)
    with open('../results/static.txt', 'w') as f:
        pprint.pprint(static, f)



