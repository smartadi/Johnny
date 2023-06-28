from __future__ import print_function
from vicon_dssdk import ViconDataStream
import argparse
import sys
import numpy as np
# Vicon definitions
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default = "localhost:801")
args = parser.parse_args()
#client = ViconDataStream.Client()
client = ViconDataStream.RetimingClient()

from digi.xbee.devices import XBeeDevice
# XBEE definitions
# Coordinator connected to server
PORT = "COM4"
BAUD_RATE = 9600
# Test data
# Router NODE ID (set to Johnny name)
REMOTE_NODE_ID = "XBEEA"






def main():
    '''
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
    '''
    client.Connect( "localhost:801" )

    # Check the version
    print( 'Version', client.GetVersion() )

    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print( 'X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis )

    #client.SetMaximumPrediction( 10 )
    print( 'Maximum Prediction', client.MaximumPrediction())

    client.UpdateFrame()

    subjectNames = client.GetSubjectNames()
    for subjectName in subjectNames:
        print(subjectName)
        segmentNames = client.GetSegmentNames(subjectName)
        for segmentName in segmentNames:
            segmentChildren = client.GetSegmentChildren(subjectName, segmentName)
            for child in segmentChildren:
                try:
                    print(child, 'has parent', client.GetSegmentParentName(subjectName, segmentName))
                except ViconDataStream.DataStreamException as e:
                    print('Error getting parent segment', e)
            print(segmentName, 'has static translation', client.GetSegmentStaticTranslation(subjectName, segmentName))
            print(segmentName, 'has static rotation( helical )',
                  client.GetSegmentStaticRotationHelical(subjectName, segmentName))
            print(segmentName, 'has static rotation( EulerXYZ )',
                  client.GetSegmentStaticRotationEulerXYZ(subjectName, segmentName))
            print(segmentName, 'has static rotation( Quaternion )',
                  client.GetSegmentStaticRotationQuaternion(subjectName, segmentName))
            print(segmentName, 'has static rotation( Matrix )',
                  client.GetSegmentStaticRotationMatrix(subjectName, segmentName))
            try:
                print(segmentName, 'has static scale', client.GetSegmentStaticScale(subjectName, segmentName))
            except ViconDataStream.DataStreamException as e:
                print('Scale Error', e)
            print(segmentName, 'has global translation', client.GetSegmentGlobalTranslation(subjectName, segmentName))
            print(segmentName, 'has global rotation( helical )',
                  client.GetSegmentGlobalRotationHelical(subjectName, segmentName))
            print(segmentName, 'has global rotation( EulerXYZ )',
                  client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName))
            print(segmentName, 'has global rotation( Quaternion )',
                  client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName))
            print(segmentName, 'has global rotation( Matrix )',
                  client.GetSegmentGlobalRotationMatrix(subjectName, segmentName))
            print(segmentName, 'has local translation', client.GetSegmentLocalTranslation(subjectName, segmentName))
            print(segmentName, 'has local rotation( helical )',
                  client.GetSegmentLocalRotationHelical(subjectName, segmentName))
            print(segmentName, 'has local rotation( EulerXYZ )',
                  client.GetSegmentLocalRotationEulerXYZ(subjectName, segmentName))
            print(segmentName, 'has local rotation( Quaternion )',
                  client.GetSegmentLocalRotationQuaternion(subjectName, segmentName))
            print(segmentName, 'has local rotation( Matrix )',
                  client.GetSegmentLocalRotationMatrix(subjectName, segmentName))


            '''
            pos_error = np.array([pos[0],pos[1],pos[2]],dtype ="float16")
            pos_error = [0,0,0] - pos_error
            rot = np.array(rot[0], rot[1], rot[2])
            rot = np.zeros
            ev = pos_error/np.linalg.norm(pos_error)
            ew = np.dot([0,0,0])'''






if __name__ == '__main__':
    main()


