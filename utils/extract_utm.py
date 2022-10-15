import pprint
  
with open('/home/aeye/Downloads/kcity_high.csv') as w:
    w.readline()
    coordinates = list()
    for alpha in w.readlines():
        data = list(map(float,alpha.split(',')[0:2]))
        coordinates.append(data)

pprint.pprint(coordinates)
