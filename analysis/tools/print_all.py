# Print all logged messages -> With id input
if __name__ == '__main__':
    _DB = input("1) Error DB, 2) Safe DB 3) Compare : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')
    
    _id = input("Enter ID : ")

    header = data.readline() # Skip first line
    for lines in data:
        if len(lines) == 17: # Last line is not parsed
            break
        parsed = lines.split()
        id = parsed[1]
        dlc = int(parsed[2])
        frames = list(parsed[3:3+dlc])

        if id == _id:
            print(frames)
