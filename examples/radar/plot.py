import uavcan
import sys
import time
import math
import matplotlib.pyplot as plt

node = uavcan.make_node(sys.argv[1])

plt.ion()
dist = [i*0.15 for i in range(60)]

def wrap_pi(x):
    if x >= math.pi:
        x -= 2*math.pi
    if x < -math.pi:
        x += 2*math.pi
    return x

def radar_cb(event):
    ampl = []
    phase = []
    real = []
    imag = []
    for x in event.message.cir:
        ampl.append((x.real**2+x.imag**2)**0.5)
        phase.append(math.atan2(x.real, x.imag))
        real.append(x.real)
        imag.append(x.imag)

    plt.subplot(211)
    plt.cla()
    plt.plot(dist, ampl, alpha=0.3)
    plt.subplot(212)
    plt.cla()
    plt.plot(dist, [wrap_pi(x-phase[1]) for x in phase], alpha=0.3)
    plt.pause(0.0001)


handle = node.add_handler(uavcan.thirdparty.com.matternet.equipment.uwb.Radar, radar_cb)

while True:
    try:
        node.spin()             # Spin forever or until an exception is thrown
    except UAVCANException as ex:
        print('Node error:', ex)
