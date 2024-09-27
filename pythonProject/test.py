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
from scipy.signal import buttap, lp2hp_zpk, bilinear_zpk, zpk2tf, butter, filtfilt
import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R
import time

a=np.array([1,2,3])
b=np.array([4,5,6])
c=a=np.array([a,b])
c=c[0:2,0:2]
print(c)
