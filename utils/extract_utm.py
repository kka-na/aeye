import pprint
  
with open('tor_test_utm.csv') as w:
    w.readline()
    coordinates = list()
    for alpha in w.readlines():
        data = alpha.split(',')[0:2]
        coordinates.append(data)

pprint.pprint(coordinates)
~                          
