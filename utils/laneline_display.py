import os
import sys
from matplotlib.style import context
import zmq
import cv2
import numpy as np

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
    line1s = md2.modelV2.laneLines[1].y
    line2s = md2.modelV2.laneLines[2].y
    print("Line1 : ")
    for point in line1s:
        print("{} ".format(point))
    print("\nLine2 : ")
    for point in line2s:
        print("{} ".format(point))
