# Working version
# To DO
# 1) Omega rolling over

import matplotlib.pyplot as plt
import numpy as np
from vicon_dssdk import ViconDataStream
import argparse
import sys
from digi.xbee.devices import XBeeDevice
from digi.xbee.devices import RemoteZigBeeDevice

from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice

from time import time
from threading import Thread, Event
from time import sleep
from threading import Lock
from scipy.signal import buttap, lp2hp_zpk, bilinear_zpk, zpk2tf, butter
import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R
event = Event()
data_lock = Lock()

# TODO: Replace with the serial port where your local module is connected to.
PORT = "COM3"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 115200


def callback_discovery_finished(status):
    if status == NetworkDiscoveryStatus.SUCCESS:
        print("  Discovery process finished successfully.")
    else:
        print("  There was an error discovering devices: %s" % status.description)


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default="localhost:801")
    args = parser.parse_args()
    client = ViconDataStream.Client()

    print(" Connecting to Vicon host")
    client.Connect(args.host)
    client.EnableSegmentData()

    hasFrame = False
    timeout = 50

    while not hasFrame:
        print('.')
        try:
            if client.GetFrame():
                hasFrame = True
            timeout = timeout - 1
            if timeout < 0:
                print('Failed to get frame')
                sys.exit()
        except ViconDataStream.DataStreamException as e:
            client.GetFrame()

    client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
    print('Get Frame Pull', client.GetFrame(), client.GetFrameNumber())
    print('Segments', client.IsSegmentDataEnabled())

    subjectName = client.GetSubjectNames()
    idata = dict()

    print(subjectName)

    for subName in subjectName:
        segmentName = client.GetSegmentNames(subName)
        print(segmentName)
        print(subName)

        print(segmentName, 'has global translation', client.GetSegmentGlobalTranslation(subName, segmentName[0]))
        pos = client.GetSegmentGlobalTranslation(subName, segmentName[0])[0]

        print(segmentName, 'has global rotation( EulerXYZ )',
              client.GetSegmentGlobalRotationEulerXYZ(subName, segmentName[0]))
        rot = client.GetSegmentGlobalRotationEulerXYZ(subName, segmentName[0])[0]
        # data.update({subName: [pos, rot]})

        idata.update({subName: np.array([pos, rot,pos,rot])})

    idata.update({'dt': 0})

    # Start Vicon
    sleep(0.1)
    print(idata)

    print('Vicon Initialised')  # Press Ctrl+F8 to toggle the breakpoint.
    return client, segmentName, subjectName, idata


def get_Vframe(client):
    return client.GetFrame()

def xbee_init():
    xbee_network = None

    xbee = XBeeDevice(PORT, BAUD_RATE)

    try:
        xbee.open()
        xbee.set_sync_ops_timeout(1)

        xbee_network = xbee.get_network()

        xbee_network.set_discovery_timeout(15)  # 15 seconds.

        xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)

        xbee_network.add_network_modified_callback(cb_network_modified)

        print("* Discover remote XBee devices...")

        xbee_network.start_discovery_process()

        while xbee_network.is_discovery_running():
            sleep(1)

        # print_nodes(xbee_network)
        remote_devices = xbee_network.get_devices()

        a = list(map(str, xbee_network.get_devices()))
        b = []
        remote_devicess=[]
        for i in range(len(a)):
            b.append(a[i][-8:])
            remote_devicess.append(xbee_network.discover_device(b[i]))

        print(b)
        remote_names = b

        print(xbee_network.get_device_by_node_id(b[0]))
        remote_device = xbee_network.discover_device(b[0])

        # xbee.send_data(remote_device," self.data")


        print(remote_devicess)
        print(remote_names)

    finally:
        if xbee_network is not None:
            xbee_network.del_discovery_process_finished_callback(callback_discovery_finished)
            xbee_network.del_network_modified_callback(cb_network_modified)

    return xbee, remote_devicess, remote_names


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
    # print("filtered")
    # print(y0)
    # print(y1)
    # print(y2)
    # print(x0)
    # print(x1)
    # print(x2)
    return np.array([y0, y1, x0, x1])


class Server:

    def __init__(self):
        self.xbee, self.remote_devicess, self.remote_names = xbee_init()  # This returns the xbee device,remote network, agents on the network

        self.cutoff_freq = 10
        self.sample_time = 0.01
        self.frame = 0
        self.fl = 0
        self.data = ","
        # vicon is the client, subjectNames are the agent names, Segment names are subgroups of each agent
        self.vicon, self.segmentName, self.subjectNames, self.mover = vicon_init()
        self.ref = dict()
        self.vfilter = dict()
        self.rfilter = dict()
        # self.xbee, self.remote_devicess, self.remote_names = xbee_init()  # This returns the xbee device,remote network, agents on the network

        # perform a check for matching vicon and xbee agents
        self.active_agents = None
        self.nagents = len(self.subjectNames)
        self.t = 0
        self.data = dict()
        self.packet = None
        self.init_var()


        # Runs on a parallel thread all the time. It works for now, I don't know how
        #self.cycle()

        self.plotterv = np.array([[0, 0, 0]])
        self.plotterr = np.array([[0, 0, 0]])
        self.johnny_update()
        # self.send_data()
        self.cycle()

    def johnny_update(self):
        # sleep(0.01)
        self.vicon.GetFrame()

        for subName in self.subjectNames:
            pos = np.asarray(self.vicon.GetSegmentGlobalTranslation(subName, subName)[0])
            rot = np.asarray(self.vicon.GetSegmentGlobalRotationEulerXYZ(subName, subName)[0])

            ref_vrot = self.ref[subName][1]
            ref_vel = self.ref[subName][0]

            vpos = [0, 0, 0]
            vrot = [0, 0, 0]

            # Filter for velocity and omega

            # print(pos)
            # print(rot)
            # print(self.vfilter[subName][0])
            # print(self.vfilter[subName])
            self.mover[subName] = np.array([pos, rot, self.vfilter[subName][0], self.rfilter[subName][0]])
            self.vfilter.update({subName: just_filter(self.vfilter[subName], pos)})
            self.rfilter.update({subName: just_filter(self.rfilter[subName], rot)})

            # = scipy_low(self.cutoff_freq, self.sample_time, self.vfilter[subName], pos)

            Rz = R.from_euler('z', rot[2], degrees=False).as_matrix()

            v = self.vfilter[subName][0]
            vr = self.rfilter[subName][0]

            vJ = np.linalg.norm(v)
            ref = [0,0,0]
            # body frame
            data_v = ref - vJ

            # data_v = np.array([[1,0,0],[0,0,0],[0,0,0]])@vb.T

            #self.plotterr = np.append(self.plotterr, [data_v], axis=0)

            ref_vrot = [0, 0, -0.1]
            print("issue")
            print(ref_vrot[2] - vr[2])

            #data_rz = int((10 * 180 * (1 / np.pi) * (ref_vrot[2] - vr[2]))) + 1800
            data_rz = int((1000 * (ref_vrot[2] - vr[2]))) + 500
            data_v = int(np.linalg.norm(data_v) * 255 / 200) + 100

            # data_rz = 1800

            print("v sent")
            print(data_v)
            print(data_rz)
            self.data.update({subName: [data_v, data_rz]})


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
            d = str(self.data[name][0]) + "," + str(self.data[name][1])
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





