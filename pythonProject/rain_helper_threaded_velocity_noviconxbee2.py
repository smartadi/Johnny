
# To DO
# 1) Manage synchronisation Vicon and Xbee agents
# 2) Vicon data sequence handling
# 3) Expose usable methods
# 4) Threading

import matplotlib.pyplot as plt
import numpy as np
import math
# from vicon_dssdk import ViconDataStream
# import argparse
# import sys
# from digi.xbee.devices import XBeeDevice
# from digi.xbee.devices import RemoteZigBeeDevice
#
# from digi.xbee.models.address import XBee64BitAddress
# from digi.xbee.models.status import NetworkDiscoveryStatus
# from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice

from time import time
from threading import Thread, Event
from time import sleep
from threading import Lock
from scipy.signal import buttap, lp2hp_zpk, bilinear_zpk, zpk2tf, butter


event = Event()
data_lock = Lock()

# TODO: Replace with the serial port where your local module is connected to.
PORT = "COM3"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 115200


# def callback_discovery_finished(status):
#     if status == NetworkDiscoveryStatus.SUCCESS:
#         print("  Discovery process finished successfully.")
#     else:
#         print("  There was an error discovering devices: %s" % status.description)


def cb_network_modified(event_type, reason, node):
    print("  >>>> Network event:")
    print("         Type: %s (%d)" % (event_type.description, event_type.code))
    print("         Reason: %s (%d)" % (reason.description, reason.code))
    if not node:
        return
    print("         Node:")
    print("            %s" % node)


def print_nodes(xb_net):
    print("\n  Current network nodes:\n    ", end='')
    if xb_net.has_devices():
        print("%s" % '\n    '.join(map(str, xb_net.get_devices())))
    else:
        print("None")


def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread

    return wrapper


def vicon_init():
    # Initialise Vicon, get initial pose of all available agents
    client = "a"

    subjectName = 'J'
    #idata = dict()

    idata = {'J': np.array([[0, 0, 0], [0, 0, 0]])}

    # Start Vicon
    sleep(0.1)
    print(idata)

    print('Vicon Initialised')  # Press Ctrl+F8 to toggle the breakpoint.
    return client, subjectName, idata


def get_Vframe(client):
    return client.GetFrame()


def xbee_init():
    xbee_network = None
    xbee = None
    remote_devicess = None
    remote_names = None

    return xbee, remote_devicess, remote_names


def winter_low(cutoff_freq, sample_time, f, m):
    """Filters a data sample based on two past unfiltered and filtered data samples.

    2nd order low pass, single pass butterworth filter presented in Winter2009.

    Parameters
    ==========
    cuttoff_freq: float
        The desired lowpass cutoff frequency in Hertz.
    sample_time: floaat
        The difference in time between the current time and the previous time.
    x0 : float
        The current unfiltered signal, x_i
    x1 : float
        The unfiltered signal at the previous sampling time, x_i-1.
    x2 : float
        The unfiltered signal at the second previous sampling time, x_i-2.
    y1 : float
        The filtered signal at the previous sampling time, y_i-1.
    y2 : float
        The filtered signal at the second previous sampling time, y_i-2.

    """
    y1 = f[0, :]
    y2 = f[1, :]
    f[1, :] = y1
    x0 = m
    x1 = f[2, :]
    x2 = f[3, :]
    sampling_rate = 1 / sample_time  # Hertz

    correction_factor = 1.0  # 1.0 for a single pass filter

    corrected_cutoff_freq = np.tan(np.pi * cutoff_freq / sampling_rate) / correction_factor  # radians

    K1 = np.sqrt(2) * corrected_cutoff_freq
    K2 = corrected_cutoff_freq ** 2

    a0 = K2 / (1 + K1 + K2)
    a1 = 2 * a0
    a2 = a0

    K3 = a1 / K2

    b1 = -a1 + K3
    b2 = 1 - a1 - K3

    y0 = a0 * x0 + a1 * x1 + a2 * x2 + b1 * y1 + b2 * y2


    return np.array([y0, y1, x0, x1])

def just_filter(f, m):
    y1 = f[0, :]
    y2 = f[1, :]
    x0 = m
    x1 = f[2, :]
    x2 = f[3, :]

    a1 = 0
    a2 = 7.8387
    a3 = -7.8387
    b1 = 1.0000
    b2 = - 1.5622
    b3 = 0.6413
    y0 = -b2 * y1 - b3 * y2 + a1 * x0 + a2 * x1 + a3 * x2
    print("filtered")
    print(y0)
    print(y1)
    print(y2)
    print(x0)
    print(x1)
    print(x2)
    return np.array([y0, y1, x0, x1])

def scipy_high(cutoff_freq, sample_time, x0, x1, x2, y1, y2):
    sample_rate = 1.0 / sample_time
    nyquist_freq = 0.5 * sample_rate
    # nyquist normalized cutoff for digital design
    Wn = cutoff_freq / nyquist_freq
    b, a = butter(2, Wn, btype='low')

    return -a[1] * y1 - a[2] * y2 + b[0] * x0 + b[1] * x1 + b[2] * x2


def scipy_low(cutoff_freq, sample_time, f, m):

    # print(f)
    # print(m)
    print(f[0, :])
    y1 = f[0, :]
    y2 = f[1, :]
    f[1, :] = y1
    x0 = m
    x1 = f[2, :]
    x2 = f[3, :]
    sample_rate = 1.0 / sample_time
    nyquist_freq = 0.5 * sample_rate
    # nyquist normalized cutoff for digital design
    Wn = cutoff_freq / nyquist_freq
    b, a = butter(2, Wn, 'low')
    print("filter")
    print(b)
    print(a)

    y0 = -a[1] * y1 - a[2] * y2 + b[0] * x0 + b[1] * x1 + b[2] * x2
    # print('data')
    # print(y0)
    # print(y1)
    # print(x0)
    # print(x1)

    return np.array([y0, y1, x0, x1])


class Server:

    def __init__(self):
        self.cutoff_freq = 10
        self.sample_time = 0.01
        self.frame = 0
        self.fl = 0
        self.data = ","
        # vicon is the client, subjectNames are the agent names, Segment names are subgroups of each agent
        self.vicon, self.subjectNames, self.mover = vicon_init()
        self.ref = dict()
        self.vfilter = dict()
        self.rfilter = dict()
        self.xbee, self.remote_devicess, self.remote_names = xbee_init()  # This returns the xbee device,remote network, agents on the network

        # perform a check for matching vicon and xbee agents
        self.active_agents = None
        #self.nagents = len(self.remote_devicess)
        self.t = 0
        self.data = dict()
        self.packet = None
        self.init_var()
        self.i = 0
        print(self.subjectNames)


        #self.send_data()
        # Runs on a parallel thread all the time. It works for now, I don't know how
        #self.cycle()
        self.plotterv = np.array([[0, 0, 0]])
        self.plotterr = np.array([[0, 0, 0]])

        #self.johnny_update()

    def johnny_update(self):
        # sleep(0.01)
        self.i = self.i + 1
        subName = 'J'

        if self.i > 0:
            print(subName)
            print("iter")
            print(self.i)

            r = 1000
            theta = self.i * 0.01

            pos = np.array([r*np.cos(2*math.pi*theta), r*np.sin(2*math.pi*theta),0]) + np.random.normal(0, 5, (1,3))
            rot = np.array([0, 0, math.pi/2 + theta]) # + np.random.normal(0, 2, (1,3))

            # pos = np.array([r * theta, r * theta, 100]) + np.random.normal(0, 5, (1, 3))
            # rot = np.array([0, 0, math.pi / 2 + theta]) + np.random.normal(0, 2, (1, 3))

            vpos = np.array([-r*2*math.pi*theta*np.sin(2*math.pi*theta), 2*math.pi*r*theta*np.cos(2*math.pi*theta), 0])
            vrot = np.array([0, 0, theta])
            print("pos")
            print(pos)

            # ref_rot = self.ref[subName][1]
            # ref_pos = self.ref[subName][0]

            # vpos = [0, 0, 0]
            # vrot = [0, 0, 0]


            # Filter for velocity and omega
            self.mover[subName] = np.array([pos, rot, self.vfilter[subName][0], self.rfilter[subName][0]])
            # self.vfilter.update({subName: scipy_low(self.cutoff_freq, self.sample_time, self.vfilter[subName], pos[0])})
            self.vfilter.update({subName: just_filter(self.vfilter[subName], pos[0])})
            # f = just_filter(self.vfilter[subName], pos[0])
            # self.vfilter[subName] = f

            # self.rfilter.update({subName: scipy_low(self.cutoff_freq, self.sample_time, self.rfilter[subName], rot[0])})

            # self.vfilter.update({subName: winter_low(self.cutoff_freq, self.sample_time, self.vfilter[subName], pos[0])})
            # = scipy_low(self.cutoff_freq, self.sample_time, self.vfilter[subName], pos)

            # data_rz = int((10 * 180 * (1 / np.pi) * (ref_rot[2]-rot[2]))) + 1800
            # data_v = (ref_pos - np.array([pos[0], pos[1], pos[2]], dtype="float16"))
            # data_v = int(np.linalg.norm(data_v) * 255 / 3000)
            # self.data.update({subName: [data_v, data_rz]})
            print("vel")
            print(self.vfilter[subName][0])

            self.plotterv = np.append(self.plotterv, [self.vfilter[subName][0]], axis=0)
            self.plotterr = np.append(self.plotterr, [self.rfilter[subName][0]], axis=0)



    def get_agents(self):
        # get agent names
        return self.subjectNames

    def get_frame(self):
        print('Vicon frame')

    def send_data(self):
        # broadcast
        i=0
        for name in self.remote_names:

            # print('iter' )
            # print(i)
            # print(self.mover[name])
            # print(name)
            # d = str(self.mover[name][0])+ ',' + str(self.mover[name][1])
            d = str(self.data[name][0])+ ',' + str(self.data[name][1])
            self.xbee.send_data_async(self.remote_devicess[i], d)
            # print('DATA')
            # print(name)
            # print(self.remote_devicess[i])
            # print(d)
            i=i+1


    @threaded
    def cycle(self):
        while (True):
            self.johnny_update()
            self.send_data()


    def get_estimate(self):
        # print('')
        # return np.array([self.t, self.mover])
        return self.mover


    def update(self, v):
        # Take a step forward on all robots simultaneously
        print("update command")

        self.ref = v

    def end(self):
        print('Motor switched off')

    def start(self):
        print('Motor switched on')

    def battery_status(self):
        print('Battery Value')

    def init_var(self):
        for name in self.subjectNames:
            # Robot.ref[name] = np.array([[0,0,0],[0,0,0]])
            self.ref.update({name: np.array([[0, 0, 0], [0, 0, 0]])})
            self.vfilter.update({name: np.zeros((4, 3))})
            self.rfilter.update({name: np.zeros((4, 3))})


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print('PyCharm')

    Robot = Server()
    # while(True):
    #
    #     data = Robot.get_estimate()
    #     print(data)
    pos=np.array([[0,0,0]])
    for j in range(100):
        Robot.johnny_update()
        print("check")
        p = Robot.get_estimate()["J"]
        print(p[0])
        pos = np.append(pos,p[0] , axis=0)

        print(Robot.plotterv[-1])

    vel = Robot.plotterv
    print(vel)


    # plt.figure(figsize=(9, 3))
    # plt.subplot(311)
    # plt.plot(pos)
    # plt.subplot(312)
    # plt.plot(pos)
    # plt.subplot(313)
    # plt.plot(pos)

    plt.plot(vel)
    plt.show()
    print(vel)



    plt.plot(pos)
    plt.plot(vel)
    plt.show()






