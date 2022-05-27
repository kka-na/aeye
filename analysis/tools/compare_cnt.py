# Compare two results 
import find_change

if __name__ == '__main__':
    ERROR = open('../data/ERROR.hst', 'r')
    SAFE = open('../data/SAFE.hst', 'r')

    error = find_change.get(ERROR)
    safe = find_change.get(SAFE)

    for key in error:
        print('____{}____'.format(key))
        print(error[key])
        print(safe[key])
    

    



