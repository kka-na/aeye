# Boilerplate for Analysis

def get_hst():
    _DB = input("1) Error DB, 2) Safe DB : ")
    if _DB == '1':
        data = open('../data/ERROR.hst', 'r')
    elif _DB == '2':
        data = open('../data/SAFE.hst', 'r')

if __name__ == '__main__':
    data = get_hst()



