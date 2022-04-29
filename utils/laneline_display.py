import zmq
import numpy as np
import matplotlib.pyplot as plt

from cereal.messaging import sub_sock, recv_one_or_none, SubMaster
from cereal.services import services

sm = SubMaster(['modelV2'], poll=['modelV2'])

addr = "192.168.0.1"
context = zmq.Context()
frame = context.socket(zmq.SUB)
frame.connect(addr)
frame.setsockopt(zmq.SUBSCRIBE, "")

modelv2 = sub_sock(context, services['modelV2'].port, addr=addr, conflate=True)
md2 = recv_one_or_none(modelv2)
if md2:
    x = md2.modelV2.laneLines[1].x
    line1s = md2.modelV2.laneLines[1].y
    line2s = md2.modelV2.laneLines[2].y

    plt.ion()
    animated_plot = plt.plot(x, line1s, 'ro', x, line2s, 'ro')[0]

    for i in range(len(x)):
        animated_plot.set_xdata(x[0:i])
        animated_plot.set_y1data(line1s[0:i])
        animated_plot.set_y2data(line2s[0:i])
        plt.draw()
        plt.pause(0.1)

    '''
    print("Line1 : ")
    for point in line1s:
        print("{} ".format(point))
    print("\nLine2 : ")
    for point in line2s:
        print("{} ".format(point))
    '''
