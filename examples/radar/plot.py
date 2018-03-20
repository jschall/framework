import uavcan
import sys
import time
import math
import matplotlib.pyplot as plt

node = uavcan.make_node(sys.argv[1])

plt.ion()
dist = [i*0.15 for i in range(60)]

def radar_cb(event):
    ampl = []
    real = []
    imag = []
    for x in event.message.cir:
        ampl.append((x.real**2+x.imag**2)**0.5)
        real.append(x.real)
        imag.append(x.imag)

    plt.cla()
    plt.plot(dist, ampl)
    plt.plot(dist, real)
    plt.plot(dist, imag)
    plt.pause(0.0001)


handle = node.add_handler(uavcan.thirdparty.com.matternet.equipment.uwb.Radar, radar_cb)

while True:
    try:
        node.spin()             # Spin forever or until an exception is thrown
    except UAVCANException as ex:
        print('Node error:', ex)
