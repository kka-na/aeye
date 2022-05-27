#Get all ids from HST

def get_hst():
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')
    return data

def get(data):
    id_list = set()
    data.readline() # Skip first line
    for lines in data:
        if len(lines) == 17: # Last line is not parsed
            break
        parsed = lines.split()
        id = parsed[1]
        id_list.add(id)

    id = sorted(id_list, key = lambda x : int(x))
    return(id)


if __name__ == '__main__':
    data = get_hst()
    data = get(data)
    print('\nNumber of IDs', len(data))


