# Scan for signal changes **ID 1215,1291,1294, 1429, 1470, 1507 not in DBC
# Complexity msg*dlc

from collections import defaultdict
import cankey
import fetch_id
import pprint

def get_hst():
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')
    return data

def reset_seek(data): # Return pointer to first line
    data.seek(0,0)
    data.readline() 
    return data

def init_dict(data):
    _dict = dict()
    for i in fetch_id.get(data):
        if i not in _dict:
            _dict[i] = [0 for _ in range(int(cankey.DLC[int(i)]))]
    reset_seek(data)
    return _dict

def get(data):
    _current = defaultdict(list)
    _changes = init_dict(data)
    _past = init_dict(data)

    for lines in data:
        if len(lines) == 17:
            break
        parsed = lines.split()
        id, dlc = parsed[1], int(parsed[2])
        frame = list(parsed[3:3+dlc])
        past = _past[id]


        for i, signal in enumerate(frame):
            if past[i] != signal:
                _changes[id][i] = _changes[id][i] + 1
            past = frame

    _changes = sorted(_changes.items(), key=lambda x : int(x[0]))
    pprint.pprint(_changes)

if __name__ == '__main__':
    data = get_hst()
    get(data)
