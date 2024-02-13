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
import time

event = Event()
data_lock = Lock()

# TODO: Replace with the serial port where your local module is connected to.
PORT = "COM4"
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
def xbee_init(names):
    xbee_network = None

    xbee = XBeeDevice(PORT, BAUD_RATE)

    try:
        xbee.open()
        xbee.set_sync_ops_timeout(1)

        xbee_network = xbee.get_network()
        remote_devicess = []

        for name in names:
            remote_devicess.append(xbee_network.discover_device(name))

        a = list(map(str, remote_devicess))
        b = []

        for i in range(len(a)):
            b.append(a[i][-8:])

        print(remote_devicess)
        print(b)

    finally:
        if xbee_network is not None:
            xbee_network.del_discovery_process_finished_callback(callback_discovery_finished)
            xbee_network.del_network_modified_callback(cb_network_modified)

    return xbee, remote_devicess, b




def vel_filter(f, m):
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


def om_filter(f, m):
    y1 = f[0, :]
    y2 = f[1, :]
    x00 = m
    x11 = f[2, :]
    x22 = f[3, :]

    a1 = 0
    a2 = 7.8387
    a3 = -7.8387
    b1 = 1.0000
    b2 = - 1.5622
    b3 = 0.6413

    if abs(x00[2] - x11[2]) > 1.5*math.pi:
        if x00[2] > x11[2]:
            x0 = x00 - 2*math.pi
        elif x00[2] < x11[2]:
            x0 = x00 + 2 * math.pi
    else:
        x0 = x00

    if abs(x11[2] - x22[2]) > 1.5*math.pi:
        if x11[2] > x22[2]:
            x1 = x11 - 2 * math.pi
        elif x11[2] < x22[2]:
            x1 = x11 + 2 * math.pi
    else:
        x1 = x11

    y0 = -b2 * y1 - b3 * y2 + a1 * x0 + a2 * x1 + a3 * x22

    # print("filtered")
    # print(y0)
    # print(y1)
    # print(y2)
    # print(x0)
    # print(x1)
    # print(x2)
    f = np.array([y0, y1, x00, x11])
    f[:, :2] = 0
    return f


class Server:

    def __init__(self):

        self.cutoff_freq = 10
        self.sample_time = 0.01
        self.frame = 0
        self.fl = 0
        self.data = ","
        # vicon is the client, subjectNames are the agent names, Segment names are subgroups of each agent
        self.vicon, self.segmentName, self.subjectNames, self.mover = vicon_init()
        self.xbee, self.remote_devicess, self.remote_names = xbee_init(self.subjectNames)  # This returns the xbee device,remote network, agents on the network

        self.ref = dict()
        self.vfilter = dict()
        self.rfilter = dict()
        self.pid_vals = dict()

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
        self.plotterx = np.array([[0, 0, 0]])
        self.plotterth = np.array([[0, 0, 0]])
        self.johnny_update()
        # self.send_data()
        # self.cycle()

        self.filtercycle()

    def johnny_update(self):
        # sleep(0.01)
        self.vicon.GetFrame()

        for subName in self.subjectNames:
            pos = np.asarray(self.vicon.GetSegmentGlobalTranslation(subName, subName)[0])
            rot = np.asarray(self.vicon.GetSegmentGlobalRotationEulerXYZ(subName, subName)[0])

            ref_vrot = self.ref[subName][1]
            ref_vel = self.ref[subName][0]

            # print(pos)
            # print(rot)
            # print(self.vfilter[subName][0])
            # print(self.vfilter[subName])
            #self.mover[subName] = np.array([pos, rot, self.vfilter[subName][0], self.rfilter[subName][0]])
            self.vfilter.update({subName: vel_filter(self.vfilter[subName], pos)})
            self.rfilter.update({subName: om_filter(self.rfilter[subName], rot)})
            self.mover[subName] = np.array([pos, rot, self.vfilter[subName][0], self.rfilter[subName][0]])
            # print(self.vfilter[subName])
            # self.rfilter.update({subName: om_filter(self.rfilter[subName], rot)})


            # = scipy_low(self.cutoff_freq, self.sample_time, self.vfilter[subName], pos)

            Rz = R.from_euler('z', rot[2], degrees=False).as_matrix()

            v = self.vfilter[subName][0]/1000  # convert to m/s
            vr = self.rfilter[subName][0]
            ev = np.linalg.norm(ref_vel) - np.linalg.norm(v)
            ew = ref_vrot[2] - vr[2]

            # proportional
            Kpv = 0.05
            Kpw = 0.01


            self.pid_vals[subName][1] = ev - self.pid_vals[subName][0]
            self.pid_vals[subName][2] = ev + self.pid_vals[subName][2]
            self.pid_vals[subName][0] = ev


            self.pid_vals[subName][4] = ew - self.pid_vals[subName][3]
            self.pid_vals[subName][5] = ew + self.pid_vals[subName][5]
            self.pid_vals[subName][3] = ew

            # PID
            Kdv = 0.001
            Kdw = 0.001

            Kiv = 0.0001
            Kiw = 0.000

            data_v = np.linalg.norm(ref_vel) + Kpv*ev + Kdv * self.pid_vals[subName][1][0] + Kiv * self.pid_vals[subName][2][0]
            data_rz = ref_vrot[2] + Kpw*ew  + Kdw * self.pid_vals[subName][4][0] + Kiw * self.pid_vals[subName][5][0]


            print('w')
            print(data_rz)
            print('v')
            print(data_v)

            data_rz = int((100 * data_rz)) + 500
            data_v = int(np.linalg.norm(data_v) * 1000) + 100

            print('conversion')
            print(data_rz)
            print(data_v)

            if (data_v > 900):
                data_v = 900

            if (data_rz > 900):
                data_rz = 900

            if (data_rz < 100):
                data_rz = 100

            # data_rz = 1800

            print("v sent")
            print(data_rz)
            print(data_v)

            self.data.update({subName: [data_v, data_rz]})

            # print(self.plotterv)
            # print(self.mover[subName][2])

            self.plotterv = np.append(self.plotterv, [self.mover[subName][2]], axis=0)
            self.plotterr = np.append(self.plotterr, [self.mover[subName][3]], axis=0)

            self.plotterx = np.append(self.plotterx, [self.mover[subName][0]], axis=0)
            self.plotterth = np.append(self.plotterth, [self.mover[subName][1]], axis=0)




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
            self.pid_vals.update({name: np.zeros((6, 1))})

    def filtercycle(self):
        for i in range(2):
            self.johnny_update()

        for name in self.subjectNames:
            self.mover[name] = np.zeros((4, 3))
            self.plotterv = np.array([[0, 0, 0]])
            self.plotterr = np.array([[0, 0, 0]])
            self.plotterx = np.array([[0, 0, 0]])
            self.plotterth = np.array([[0, 0, 0]])

        eps = 10
        while eps > 0.1:
            eps = 0
            self.johnny_update()

            for name in self.subjectNames:
                eps = eps + np.linalg.norm(self.vfilter[name][0]) + np.linalg.norm(self.rfilter[name][0])
                print(eps)

                self.mover[name] = np.zeros((4, 3))
                self.plotterv = np.array([[0, 0, 0]])
                self.plotterr = np.array([[0, 0, 0]])
                self.plotterx = np.array([[0, 0, 0]])
                self.plotterth = np.array([[0, 0, 0]])



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print('PyCharm')

    Robot = Server()

    pos = np.array([[0, 0, 0,0,0,0]])
    vel = np.array([[0, 0, 0,0,0,0]])

    rot = np.array([[0, 0, 0, 0, 0, 0]])
    rvel = np.array([[0, 0, 0, 0, 0, 0]])

    sp = []



    for j in range(1000):
        Robot.johnny_update()


        # print("check")
        p = np.zeros((1,6))
        v = np.zeros((1,6))

        r = np.zeros((1, 6))
        a = np.zeros((1, 6))

        i=0
        for name in Robot.subjectNames:
            est = Robot.get_estimate()[name]

            p[0,3*i:3*i+3] = est[0]
            v[0,3*i:3*i+3] = est[2]

            r[0, 3 * i:3 * i + 3] = est[1]
            a[0, 3 * i:3 * i + 3] = est[3]

            Robot.ref[name] = np.array([[0.1, 0.0, 0.0], [0.0, 0.0, 0.00]])


            i = i + 1
        Robot.send_data()
        # print(v)
        pos = np.append(pos, [p[0]], axis=0)
        vel = np.append(vel, [v[0]], axis=0)
        rot = np.append(rot, [r[0]], axis=0)
        rvel = np.append(rvel, [a[0]], axis=0)


    sp = np.linalg.norm(vel,2, axis = 1)


    vb = Robot.plotterv
    ve = Robot.plotterr

    plt.plot(vb)
    plt.show()

    plt.plot(sp)
    plt.show()

    plt.plot(ve)
    plt.show()

    plt.plot(pos)
    plt.show()

    plt.plot(rot)
    plt.show()





