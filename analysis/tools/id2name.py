import cankey

if __name__ == '__main__':
    while 1:
        id = int(input('Enter ID : '))
        if id in cankey.ID:
            print(cankey.ID[id])
        else:
            print('No such id')
