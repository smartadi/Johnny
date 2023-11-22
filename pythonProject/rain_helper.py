# To DO
# 1) Manage synchronisation Vicon and Xbee agents
# 2) Vicon data sequence handling
# 3) Expose usable methods
# 4) Threading


import numpy as np
import time
from vicon_dssdk import ViconDataStream
import argparse
import sys
from digi.xbee.devices import XBeeDevice


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
    print(subjectName)
    segmentName = client.GetSegmentNames(subjectName[0])
    print(segmentName)

    print(segmentName, 'has global translation', client.GetSegmentGlobalTranslation(subjectName[0], segmentName[0]))
    pos = client.GetSegmentGlobalTranslation(subjectName[0], segmentName[0])[0]

    print(segmentName, 'has global rotation( EulerXYZ )',
          client.GetSegmentGlobalRotationEulerXYZ(subjectName[0], segmentName[0]))
    rot = client.GetSegmentGlobalRotationEulerXYZ(subjectName[0], segmentName[0])[0]

    # Start Vicon
    time.sleep(0.5)

    print('Vicon Initialised')  # Press Ctrl+F8 to toggle the breakpoint.
    return client, segmentName, subjectName

def get_Vframe(client):
    return client.GetFrame()


def xbee_init():
    # Coordinator connected to server
    # CHECK THIS
    PORT = "COM4"
    BAUD_RATE = 115200
    # Test data
    # Router NODE ID (set to Johnny name)
    REMOTE_NODE_ID = "Johnny03"
    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        xbee_network = device.get_network()
        print(xbee_network)
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

    except:
        print("No connection")

    # Start Xbeagent, heading, omegae
    #time.sleep(1)
    print('Xbee Initialised')
    status = 0
    return device, remote_device


class Server:

    def __init__(self):
        self.ref = 0
        self.data = None
        self.vicon, self.segmentName, self.subjectName = vicon_init()             # This returns the vicon client
        self.xbee, self.remote = xbee_init()  # This returns the xbee device and remote network
        self.active_agents  = None
        self.agents = None
        self.t = 0
        self.data = None
        self.packet = None

        self.vicon_update()

        self.send_data()
        self.cycle()

    def vicon_update(self):
        # time.sleep(0.1)
        self.vicon.GetFrame()
        pos = self.vicon.GetSegmentGlobalTranslation(self.subjectName[0], self.segmentName[0])[0]
        rot = self.vicon.GetSegmentGlobalRotationEulerXYZ(self.subjectName[0], self.segmentName[0])[0]
        data_rz = int((10 * 180 * (1 / np.pi) * rot[2])) + 1800
        print(data_rz)
        ref = np.array([0, 0, 0]) #reference point:
        data_v = (ref - np.array([pos[0], pos[1], pos[2]], dtype="float16"))
        data_v = int(np.linalg.norm(data_v) * 255 / 3000)
        print(data_v)
        # print(data_r)
        # data_c = str(data[0]) + "\n" + str(data[1]) + "\n"+str(data[2]) + "\n"
        self.data = str(data_v) + "," + str(data_rz)

    def get_agents(self):
        # get agent names
        print('Agent names')

    def get_frame(self):
        print('Vicon frame')

    def send_data(self):
        self.xbee.send_data(self.remote, self.data)

        #time.sleep(0.2)
        #self.packet = np.linalg.norm(self.data) +  self.ref
        print('xbee_command sent')

    def cycle(self):
        for i in range(100):
            self.vicon_update()
            self.send_data()



    def get_estimate(self):
        print('')
        return np.array([self.t, self.data])

    def update(self, velocity):
        # Take a step forward on all robots simultaneously
        print("update command")
        self.ref = velocity
        return self.ref

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


