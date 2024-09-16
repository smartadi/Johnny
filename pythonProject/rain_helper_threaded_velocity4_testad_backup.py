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
from scipy.signal import buttap, lp2hp_zpk, bilinear_zpk, zpk2tf, butter, filtfilt
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
    timeout = 1000

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


def lpf(f, m):
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
    #print("filtered")
    # print(y0)
    # print(y1)
    # print(y2)
    # print(x0)
    # print(x1)
    # print(x2)
    return np.array([y0, y1, x0, x1])

def vel_filter2(f, m):
    # y1 = m
    # y2 = f[0, :]
    # y3 = f[1, :]
    # y4 = f[2, :]

    a1 = 0.101
    a2 = 0.462
    a3 = -0.462
    a4 = -0.101

    # a1 = -0.124
    # a2 = 0.248
    # a3 = 0.248
    # a4 = 0.248
    # a5 = -0.124

    y0 = a1 * m + a2 * f[0,:] + a3 * f[1,:] + a4 * f[2,:] # + a5 * f[3,:]
    # f[3, :] = f[2, :]
    f[2, :] = f[1, :]
    f[1, :] = f[0, :]
    f[0, :] = m
    #print("filtered")
    # print(y0)
    # print(y1)
    # print(y2)
    # print(x0)
    # print(x1)
    # print(x2)
    return y0, f
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
    #print("filtered")
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

def calculate_velocity(positions, timestamps):
    velocities = {}
    for subName, pos_list in positions.items():
        pos_array = np.array(pos_list)
        time_array = np.array(timestamps)

        # Calculate velocity as derivative of position
        dt = np.diff(time_array)
        dx = np.diff(pos_array, axis=0)
        velocity = np.divide(dx, dt[:, None], out=np.zeros_like(dx), where=dt[:, None]!=0)  # Avoid division by zero

        # Apply Butterworth low-pass filter
        b, a = butter(N=3, Wn=0.05)  # adjust as needed
        filtered_velocity = np.zeros_like(velocity)
        for i in range(3):  # filter for each spatial dimension (x, y, z)
            filtered_velocity[:, i] = filtfilt(b, a, velocity[:, i])

        velocities[subName] = filtered_velocity
    return velocities


def calculate_raw_velocity(pos, prevpos, prevtime):
    #x_cur, y_cur, z_cur = pos
    x_cur = pos[0]
    y_cur = pos[1]
    z_cur = pos[2]
    #x_prev, y_prev, z_prev = prevpos
    x_prev = prevpos[0]
    y_prev = prevpos[1]
    z_prev = prevpos[2]

    delta_t = prevtime - time.time()

    vx = (x_cur - x_prev) / delta_t
    vy = (y_cur - y_prev) / delta_t
    vz = (z_cur - z_prev) / delta_t

    velocity_magnitude = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    # print("type = ", type(velocity_magnitude))
    return velocity_magnitude

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
        self.pdata = dict()

        self.prevpos = [0,0,0]
        self.prevtime = time.time()
        self.rawvel = 0

        # self.xbee, self.remote_devicess, self.remote_names = xbee_init()  # This returns the xbee device,remote network, agents on the network

        # perform a check for matching vicon and xbee agents
        self.active_agents = None
        self.nagents = len(self.subjectNames)
        self.t = 0
        self.data = dict()
        self.packet = None
        self.init_var()
        self.pos2plot=np.zeros([1,3])
        self.stop = False


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
        self.live_update2(True)
        self.plot = True
        self.cycle()

    def johnny_update(self):
        # sleep(0.01)
        self.vicon.GetFrame()

        for subName in self.subjectNames:
            pos = np.asarray(self.vicon.GetSegmentGlobalTranslation(subName, subName)[0])
            rot = np.asarray(self.vicon.GetSegmentGlobalRotationEulerXYZ(subName, subName)[0])

            #self.pos2plot=np.append(self.pos2plot, (pos),axis=0)
            #print('Pos=',pos)
            ref_vrot = self.ref[subName][1]
            ref_vel = self.ref[subName][0]

            # rawvel = calculate_raw_velocity(pos, self.prevpos, self.prevtime)
            # print(rawvel)

            # self.prevpos = pos
            # self.prevtime = time.time()
            # self.rawvel = rawvel

            # self.vfilter.update({subName: vel_filter2(self.vfilter[subName], pos)})
            vf , pd = vel_filter2(self.pdata[subName], pos)
            self.pdata.update({subName: pd})
            print(self.pdata[subName])

            self.rfilter.update({subName: om_filter(self.rfilter[subName], rot)})
            self.mover[subName] = np.array([pos, rot, vf, self.rfilter[subName][0]])

            Rz = R.from_euler('z', rot[2], degrees=False).as_matrix()

            # v = self.vfilter[subName][0]/1000  # convert to m/s
            v = vf/1000  # convert to m/s
            #print("Vel=",v*1000)
            vr = self.rfilter[subName][0]
            # ev = np.linalg.norm(ref_vel) - np.linalg.norm(v)
            ev = ref_vel[0] - np.linalg.norm(v[:2])
            ew = ref_vrot[2] - vr[2]





            # data_v = np.linalg.norm(ref_vel) + Kpv*ev
            # data_rz = ref_vrot[2] + Kpw * ew

            self.pid_vals[subName][1] = ev - self.pid_vals[subName][0]
            self.pid_vals[subName][2] = ev + self.pid_vals[subName][2]
            self.pid_vals[subName][0] = ev

            self.pid_vals[subName][4] = ew - self.pid_vals[subName][3]
            self.pid_vals[subName][5] = ew + self.pid_vals[subName][5]
            self.pid_vals[subName][3] = ew

            # proportional
            Kpv = 0.2
            Kpw = 0.1  # 0.2

            # PID
            Kdv = 0.0 #0.01
            Kdw = 0.0 #0.01

            #if self.chk==1 and ev<1:
             #   self.pid_vals[subName][2][0]=0
              #  self.chk=2

            Kpw = 0 #0.2


            Kiv = 0.0 #0.01
            Kiw = 0.0 #0.005

            Kkv = 1
            Kkw = 1

            data_v = (Kkv*np.linalg.norm(ref_vel[0])) + (Kpv*ev + Kdv * self.pid_vals[subName][1][0] + Kiv * self.pid_vals[subName][2][0])/500
            data_rz = Kkw*ref_vrot[2] + Kpw*ew + Kdw * self.pid_vals[subName][4][0] + np.minimum(Kiw*1,Kiw * self.pid_vals[subName][5][0])
            #print('ev=',ev)
            #print('iev=',self.pid_vals[subName][2][0])
            #print('dev=',self.pid_vals[subName][1][0])

            #print('w')
            #print(data_rz)
            print('v')
            print(v)
            print("data_v")
            print(data_v)

            data_rz = int((100 * data_rz)) + 500
            data_v = int(np.linalg.norm(data_v) * 100) + 100

            # print('conversion')
            # print(data_rz)
            print(data_v)

            if (data_v > 900):
                data_v = 900

            if (data_rz > 900):
                data_rz = 900

            if (data_rz < 100):
                data_rz = 100

            # data_rz = 1800

            # print("v sent")
            # print(data_rz)
            print(data_v)

            self.data.update({subName: [data_v, data_rz]})

            # print(self.plotterv)
            # print(self.mover[subName][2])

            self.plotterv = np.append(self.plotterv, [self.mover[subName][2]], axis=0)
            self.plotterr = np.append(self.plotterr, [self.mover[subName][3]], axis=0)

            self.plotterx = np.append(self.plotterx, [self.mover[subName][0]], axis=0)
            self.plotterth = np.append(self.plotterth, [self.mover[subName][1]], axis=0)


    @threaded
    def live_update2(self, blit=False):
        x = np.linspace(0, 50., num=100)
        Vx = np.zeros(x.shape)
        Vr = np.zeros(x.shape)
        Vxx = np.zeros(x.shape)
        Vyy = np.zeros(x.shape)
        Vth = np.zeros(x.shape)

        fig = plt.figure(figsize=(15,5))
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        #ax3 = fig.add_subplot(5, 1, 3)
        #ax4 = fig.add_subplot(5, 1, 4)
        #ax5 = fig.add_subplot(5, 1, 5)

        line1, = ax1.plot([], lw=3)
        line2, = ax2.plot([], lw=3)
        #line3, = ax3.plot([], lw=3)
        #line4, = ax4.plot([], lw=3)
        #line5, = ax5.plot([], lw=3)


        ax1.set_xlim(x.min(), x.max())
        ax1.set_ylim([0, 5])

        ax2.set_xlim(x.min(), x.max())
        ax2.set_ylim([-3, 3])

        #ax3.set_xlim(x.min(), x.max())
        #ax3.set_ylim([-1500, 1500])

        #ax4.set_xlim(x.min(), x.max())
        #ax4.set_ylim([-1500, 1500])


        #ax5.set_xlim(x.min(), x.max())
        #ax5.set_ylim([-10, 10])

        fig.canvas.draw()  # note that the first draw comes before setting data

        if blit:
            # cache the background
            ax1background = fig.canvas.copy_from_bbox(ax1.bbox)
            ax2background = fig.canvas.copy_from_bbox(ax2.bbox)
            #ax3background = fig.canvas.copy_from_bbox(ax3.bbox)
            #ax4background = fig.canvas.copy_from_bbox(ax4.bbox)
            #ax5background = fig.canvas.copy_from_bbox(ax5.bbox)

        plt.show(block=False)

        # t_start = time.time()
        k = 0.

        #for i in np.arange(10000):
        while(self.plot == True):
            from scipy.ndimage import shift

            v = self.mover[self.subjectNames[0]][2]  # convert to 10cm/s
            # v = self.rawvel  # convert to m/s
            r = self.mover[self.subjectNames[0]][3]

            xx = self.mover[self.subjectNames[0]][0]
            th = self.mover[self.subjectNames[0]][1]

            #print("vel:::")
            #print(v)

            #print("rot:::")
            #print(r)

            #print("pos:::")
            #print(xx)

            #print("th:::")
            #print(th)
            #vr = self.rfilter[subName][0]
            #x =
            Vx = np.concatenate((Vx[1:],[np.linalg.norm(v)]))
            Vr = np.concatenate((Vr[1:],[r[2]]))
            Vxx = np.concatenate((Vxx[1:],[xx[0]]))
            Vyy = np.concatenate((Vyy[1:],[xx[1]]))
            Vth = np.concatenate((Vth[1:],[th[2]]))


            line1.set_data(x, Vx)
            line2.set_data(x, Vr)
            #line3.set_data(x, Vxx)
            #line4.set_data(x, Vyy)
            #line5.set_data(x, Vth)
            #line2.set_data(x, np.sin(x / 3. + k))
            # tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps=((i + 1) / (time.time() - t_start)))
            # text1.set_text(tx)
            # text2.set_text(tx)
            # print tx
            k += 0.11
            if blit:
                # restore background
                fig.canvas.restore_region(ax1background)
                fig.canvas.restore_region(ax2background)
                #fig.canvas.restore_region(ax3background)
                #fig.canvas.restore_region(ax4background)
                #fig.canvas.restore_region(ax5background)

                # redraw just the points
                ax1.draw_artist(line1)
                ax2.draw_artist(line2)
                #ax1.draw_artist(line3)
                #ax2.draw_artist(line4)
                #ax1.draw_artist(line5)


                # fill in the axes rectangle
                fig.canvas.blit(ax1.bbox)
                fig.canvas.blit(ax2.bbox)
                #fig.canvas.blit(ax3.bbox)
                #fig.canvas.blit(ax4.bbox)
                #fig.canvas.blit(ax5.bbox)

            else:

                fig.canvas.draw()

            fig.canvas.flush_events()


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
            d = str(self.mover[name][0])+ ',' + str(self.mover[name][1])
            d = str(self.data[name][0]) + "," + str(self.data[name][1])
            # print('d=',d)
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

            if self.stop == True:
                break


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
            self.vfilter.update({name: np.zeros((6, 3))})
            self.pdata.update({name: np.zeros((3, 3))})
            self.rfilter.update({name: np.zeros((4, 3))})
            self.pid_vals.update({name: np.zeros((6, 1))})

    def filtercycle(self):
        for i in range(100):
            self.johnny_update()

        for name in self.subjectNames:
            self.mover[name] = np.zeros((4, 3))
            self.plotterv = np.array([[0, 0, 0]])
            self.plotterr = np.array([[0, 0, 0]])
            self.plotterx = np.array([[0, 0, 0]])
            self.plotterth = np.array([[0, 0, 0]])

        veps = 1
        reps = 1
        while veps > 0.1 and reps > 0.1:
            #eps = 0
            self.johnny_update()

            for name in self.subjectNames:
                # eps = eps + np.linalg.norm(self.vfilter[name][0]) + np.linalg.norm(self.rfilter[name][0])
                veps = np.linalg.norm(self.vfilter[name][0])
                reps = np.linalg.norm(self.rfilter[name][0])
                # print(veps)
                # print(reps)

        # self.mover[name] = np.zeros((4, 3))
        # self.plotterv = np.array([[0, 0, 0]])
        # self.plotterr = np.array([[0, 0, 0]])
        # self.plotterx = np.array([[0, 0, 0]])
        # self.plotterth = np.array([[0, 0, 0]])



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print('PyCharm')

    Robot = Server()

    pos = np.array([[0, 0, 0,0,0,0]])
    vel = np.array([[0, 0, 0,0,0,0]])

    rot = np.array([[0, 0, 0, 0, 0, 0]])
    rvel = np.array([[0, 0, 0, 0, 0, 0]])

    sp = []

    eps = 100
    t0 = time.time()
    print("start")
    print(t0 - time.time())
    D = 100

    while( time.time()-t0 < D):
        #print("time")
        # print(t0 - time.time())

        t = time.time()
        T = time.time()-t0
        # while(time.time()-t<0.05):
        #     # print("loop")
        #     # print( time.time()-t)
        #     Robot.johnny_update()
        #     Robot.send_data()

        # print(T)

        for name in Robot.subjectNames:
            a = Robot.get_estimate()
            # print(a)
            p = a[name][0][:2]
            v = a[name][2][0]  # convert to m/s
            r = a[name][1][2]
            w = a[name][3][2]

            # print(v)

            # print(r)

            Kv = 1
            Kw = 1
            ep = [0,0] - p
            ref_v = Kv*np.linalg.norm(ep)
            ref_w = Kw*(np.arctan2(ep[1],ep[0])-r)
            #print(r)
            #print(np.arctan2(ep[1],ep[0]))
            # print(ref_v)
            # print(ref_w)
            # figure 8
            # wd = 1
            # vx = 0.5*wd*math.sin(wd*T)
            # vy = 0.5*wd*math.cos(wd*T)
            # v = math.sqrt(vx**2 + vy**2)
            #
            # Robot.ref[name] = np.array([[v, 0.0, 0.0], [0.0, 0.0, wd]])
            # print(v)

            # circle
            wd = 0
            # vx = 0.5*wd*math.sin(wd*T)
            # vy = 0.5*wd*math.cos(wd*T)
            v = 0
            ref_v = 1.5
            ref_w = 0

            Robot.ref[name] = np.array([[ref_v, 0.0, 0.0], [0.0, 0.0, ref_w]])
    Robot.plot = False
    Robot.stop = True



    # stop the robot
    for name in Robot.subjectNames:
        Robot.ref[name] = np.array([[0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    t = time.time()
    while (time.time() - t < 1):
        Robot.johnny_update()
        Robot.send_data()

        # live_update_demo(False) # 28 fps

    vb = Robot.plotterv
    ve = Robot.plotterr
    pos=Robot.plotterx


    sp = np.linalg.norm(vb, axis=1)

    plt.plot(vb)
    plt.title("velocity")
    plt.show()

    plt.plot(pos)
    plt.title("Position")
    plt.show()


    plt.plot(ve)
    plt.title("omega")
    plt.show()

    kern=np.ones(50)/50
    smooth_sp=np.convolve(sp,kern, mode='valid')
    plt.plot(sp)
    lsp=len(sp)

    v=1

    des_vel=100*v*np.ones(lsp)
    plt.plot(des_vel)
    plt.title("Speed")
    plt.show()

    plt.plot(Robot.pos2plot[:,0])
    plt.show()







