from __future__ import print_function
from vicon_dssdk import ViconDataStream
import argparse
import sys
import numpy as np
# Vicon definitions
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default = "localhost:801")
args = parser.parse_args()
client = ViconDataStream.Client()

from digi.xbee.devices import XBeeDevice
# XBEE definitions
# Coordinator connected to server
PORT = "COM4"
BAUD_RATE = 9600
# Test data
# Router NODE ID (set to Johnny name)
REMOTE_NODE_ID = "XBEEA"






def main():
    print(" Connecting to Vicon host")
    client.Connect( args.host )
    client.EnableSegmentData()
    HasFrame = False
    timeout = 50
    while not HasFrame:
        print( '.' )
        try:
            if client.GetFrame():
                HasFrame = True
            timeout=timeout-1
            if timeout < 0:
                print('Failed to get frame')
                sys.exit()
        except ViconDataStream.DataStreamException as e:
            client.GetFrame()

    client.SetStreamMode( ViconDataStream.Client.StreamMode.EClientPull )
    print( 'Get Frame Pull', client.GetFrame(), client.GetFrameNumber() )
    print( 'Segments', client.IsSegmentDataEnabled() )

    subjectName = client.GetSubjectNames()
    print(subjectName)
    segmentName = client.GetSegmentNames(subjectName[0])
    print(segmentName)

    print(segmentName, 'has global translation', client.GetSegmentGlobalTranslation(subjectName[0], segmentName[0]))
    pos = client.GetSegmentGlobalTranslation(subjectName[0], segmentName[0])[0]




    data = (10*(np.array([pos[0],pos[1],pos[2]],dtype ="float16")+np.array([1500,1500,1500]))).astype(int)
    print(data.dtype)
    print(data)
    data_c = str(data[0]) + "\n" + str(data[1]) + "\n"+str(data[2]) + "\n"
    print(data_c)
    print(" +---------------------------------------------------+")
    print(" | XBee Python Library Connect to Echo Server Sample |")
    print(" +---------------------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        print(xbee_network)
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

        print("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), data_c))

        device.send_data(remote_device, data_c)
        print("Sent")



    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    main()


