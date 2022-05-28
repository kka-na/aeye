import cankey

if __name__ == '__main__':
    while 1:
        name = input('Enter name : ')
        name = name.upper()

        if name in cankey.NAME:
            print(cankey.NAME[name])
        else:
            print('No such name')
