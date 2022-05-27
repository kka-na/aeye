#Get all ids from HST
if __name__ == '__main__':
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')

    id_list = set()

    data.readline() # Skip first line
    for lines in data:
        if len(lines) == 17: # Last line is not parsed
            break
        parsed = lines.split()
        id = parsed[1]
        id_list.add(id)

    data = sorted(id_list, key = lambda x : int(x))
    print(data)
    print('\nNumber of IDs', len(data))


