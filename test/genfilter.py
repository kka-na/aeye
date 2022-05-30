import cankey

data = cankey._SCC

def printer(ind, low, high):
    print('<DIR Name="FMT{}">'.format(ind))
    print(' <STR Name="Name">StopFilter</STR>')
    print(' <INT Name="Low">{}</INT>'.format(low))
    print(' <INT Name="High">{}</INT>'.format(high))
    print(' <INT Name="ApplyExt">0</INT>')
    print(' <INT Name="ApplyStd">1</INT>')
    print('</DIR>')

prev = 0 
for i, key in enumerate(data):
    printer(i, prev, int(data[key])-1)
    prev = int(data[key])+1
