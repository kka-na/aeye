# Parsing hst files

if __name__ == '__main__':
    with open('../data/SAFE.hst', 'r') as data:
        header = data.readline()
        for lines in data:
            parsed = lines.split()
            id = parsed[1]
            dlc = int(parsed[2])
            frames = list(parsed[3:3+dlc])
            print(id)
            print(frames)

