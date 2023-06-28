# To DO
# 1) Manage synchronisation Vicon and Xbee agents
# 2) Vicon data sequence handling
# 3) Expose usable methods
# 4) Threading


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
    ## Initialise Vicon, get initial pose of all available agents
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

    client.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)
    print('Get Frame Pull', client.GetFrame(), client.GetFrameNumber())
    print('Segments', client.IsSegmentDataEnabled())

    subjectName = client.GetSubjectNames()
    l = len(subjectName)
    data = []

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
        data.append([subName, pos, rot])

    # Start Vicon
    sleep(0.5)
    print(data)

    print('Vicon Initialised')  # Press Ctrl+F8 to toggle the breakpoint.
    return client, segmentName, subjectName, data


def get_Vframe(client):
    return client.GetFrame()


def xbee_init2():
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

        # print(b)
        remote_names = b
        #
        # print(xbee_network.get_device_by_node_id(b[1]))
        # remote_device = xbee_network.discover_device(b[1])
        #
        # xbee.send_data(remote_device, "self.data")
        # xbee.send_data(xbee_network.get_devices()[0], "self.data")
        #
        # xbee.send_data(remote_device, "self.data")
        # xbee.send_data(xbee_network.get_devices()[1], "self.data")
        #
        # print(remote_device)




    finally:
        if xbee_network is not None:
            xbee_network.del_discovery_process_finished_callback(callback_discovery_finished)
            xbee_network.del_network_modified_callback(cb_network_modified)

    return xbee, remote_devicess, remote_names


def xbee_init():
    # # Coordinator connected to server
    # PORT = "COM3"
    # BAUD_RATE = 115200
    # # Test data
    # # Router NODE ID (set to Johnny name)
    # REMOTE_NODE_ID = "Johnny03"
    # device = XBeeDevice(PORT, BAUD_RATE)
    # print(device)

    xbee_network = None

    xbee = XBeeDevice(PORT, BAUD_RATE)

    try:

        xbee.open()
        xbee.set_sync_ops_timeout(0.1)

        xbee_network = xbee.get_network()

        xbee_network.set_discovery_timeout(15)  # 15 seconds.

        xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)

        xbee_network.add_network_modified_callback(cb_network_modified)

        print("* Discover remote XBee devices...")

        xbee_network.start_discovery_process()

        while xbee_network.is_discovery_running():
            time.sleep(1)

        # print_nodes(xbee_network)
        # print("%s" % '\n    '.join(map(str, xbee_network.get_devices())))
        print(list(map(str, xbee_network.get_devices())))

        # if remote_device is None:
        #    print("Could not find the remote device")
        #    exit(1)

    except:
        print("No connection")

    # print(remote_device.read_device_info())

    # Start Xbeagent, heading, omegae
    sleep(1)
    print('Xbee Initialised')
    status = 0
    # print(agents.node_id)
    # print(agents.x64bit_addr)
    # print(xbee_network.get_devices())

    # print(remote_device)
    # print(aa)

    # print_nodes()

    return xbee


class Server:

    def __init__(self):
        self.ref = np.array([0, 0, 0])
        self.data = ","
        # vicon is the client, subjectNames are the agent names, Segement names are subgroups of each agent
        #self.vicon, self.segmentName, self.subjectNames, self.mover = vicon_init()
        pos=[0,0,0]
        rot = [0, 0, 0]
        self.mover = [pos,rot]
        self.xbee, self.remote_devicess, self.remote_names = xbee_init2()  # This returns the xbee device,remote network, agents on the network
        l = len(self.remote_devicess)
        # perform a check for matching vicon and xbee agents
        self.active_agents = None
        self.nagents = l
        self.t = 0
        self.data = None
        self.packet = None
        # self.mover = [self.subjectNames, np.zeros((l, 3)), np.zeros((l, 3))] # list of agent reference positions
        # xbee_init2()
        self.fl = 0
        self.johnny_update()

        self.send_data()


        # Runs on a parallel thread all the time. It works for now, I don't know how
        # self.cycle()

    def johnny_update(self):
        sleep(0.2)
        if self.fl > 1000:
            self.fl = 0


        self.fl = self.fl + 1
        #self.vicon.GetFrame()
        #for subName in self.subjectNames:
        #     print(subName)
        # for rname in self.remote_names:
        #    print(rname)

        # print(self.subjectNames[0])
        # print(self.segmentName[0])
        # pos = self.vicon.GetSegmentGlobalTranslation(self.subjectNames[0], self.segmentName[0])[0]
        pos = [100,200,0]

        rot = [0,0,0.5]

        data_rz = int((10 * 180 * (1 / np.pi) * (-1)**self.fl)) + 1800

        print(data_rz)
        ref = self.ref
        data_v = (ref - np.array([pos[0], pos[1], pos[2]], dtype="float16"))
        data_v = int(np.linalg.norm(data_v) * 255 / 3000)
        # print(data_v)
        # print(data_r)
        # data_c = str(data[0]) + "\n" + str(data[1]) + "\n"+str(data[2]) + "\n"
        self.data = str(data_v) + "," + str(data_rz)
        #for i in range(10):
        #    self.data = self.data + str(data_v) + "," + str(data_rz)

    def get_agents(self):
        # get agent names
        return self.subjectNames

    def get_frame(self):
        print('Vicon frame')

    def send_data(self):
        # broadcast
        print(self.data)
        for i in range(self.nagents):
            self.xbee.send_data_async(self.remote_devicess[i], self.data)

    @threaded
    def cycle(self):
        while (True):
            self.johnny_update()
            # self.send_data()

    def get_estimate(self):
        print('')
        return np.array([self.t, self.data])

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


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print('PyCharm')

    Robot = Server()

    for i in range(1000):
        # Robot.update(100*i*np.array([1, 1, 1]))
        Robot.johnny_update()
        Robot.send_data()
        #sleep(0.1)

    print(Robot.data)
