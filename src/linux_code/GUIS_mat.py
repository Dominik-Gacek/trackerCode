
import sys, os, math, serial, datetime, time, copy
import numpy as np
import random
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy.optimize import minimize
import tkinter
from tkinter.scrolledtext import ScrolledText

#########IMPORTING FILE TO DEVICE
#return false for 0
def _o(x):
    if x == 0: return False
    return True

#load from USB0 INTO data0.bin
#not actually a binary file, half as efficient as each char stores half byte
class Loader:
    def __init__(self):
        pass

    def renamef(self, names,name = 0):
        if 'data'+str(name+1)+'.bin' in names:
            self.renamef(names, name+1)
        os.rename('data'+str(name)+'.bin', 'data'+str(name+1)+'.bin')

    def init_uart(self):
        port = ""
        baudrate = 115200
        timeout = 5.0
        fnames = []
        if sys.platform in ["linux","linux2"]:
            directory = os.fsencode(os.getcwd())
            for file in os.listdir(directory):
                fnames.append(os.fsdecode(file))
            if 'data0.bin' in fnames:self.renamef(fnames)

            port = '/dev/ttyUSB0'
        elif sys.platform == "win32":
            a = 3 #TODO
            port = f'COM{a}'
        else:os._exit(0)
        f = open("data0.bin", 'ab+');
        t = time.monotonic_ns()+2000000000
        while True:
            if time.monotonic_ns()> t:
                return 1
            try:
                ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                print("serial is open: ", ser.is_open)
                x = ser.write(b'bbbbbbb')
                ser.flush()
                break
            except:continue

        while True:
            try:x = ser.read(256)
            except:return 1
            f.write(x)
            if not x:break
        ser.close()
        f.close()

        #delete debug lines, sometimes uart is not flushed and saves debug info to file, alternaively debug can be disabled at start
        lines = []
        with open("data0.bin", 'r') as f:
            while True:
                    c = f.readline()
                    if not c:break
                    if c[0] not in ['I', 'E']:
                        lines.append(c)
        with open(f"data0.bin", "w") as f:
            f.writelines(lines)

        return 0


ERROR_SCALE = 100 # scale of the error value of results, this makes the sphere cover all possible positions
IMAGESCALE = 100/1

###########Collecting data from files and storing
class nodeData:
    def __init__(self):
        self.recv = {}
        '''
        recv ={
            "str"mac:{
                "int"time:(x, y),
                "int"time:(x, y)
            }
            ,"str"mac{
                'const':(x, y),
                'offset':"float"t
            }
        }
        '''
        self.nodes = {}
        '''
        nodes ={
            "str"node:{
                "int"time:{
                    "str"recvid:"int"rssi,
                    "str"recvid:"int"rssi
                },
                "int"time:{
                    "str"recvid:"int"rssi,
                    "str"recvid:"int"rssi
                }
            },
            "str"node...
        }
        '''

    def addData(self, name:str, recvid:str, time:float, rssi:int):#add node data
        ntime = round(time*16)
        if name not in self.nodes:
            self.nodes[name] = {ntime:{recvid:rssi}}
        else:
            if ntime not in self.nodes[name]:
                self.nodes[name][ntime] = {recvid:rssi}
            else:
                if recvid not in self.nodes[name][ntime]:
                    self.nodes[name][ntime][recvid] = rssi
                else:
                    pass

    def ReadRecv(self, st, i):
        try:
            st +=' '
            ch = ""
            t = True
            for j in st:
                if j ==' ':
                    if t:x = float(ch);ch="";t=False
                    else:y = float(ch);break
                ch+=j
            self.addRecvPos(f'recv{i}', [x, y])
            return 0
        except Exception as e: print(e);return 1
        return 1

    def ReadTimes(self, st):
        try:
            st +=' '
            ch = ""
            t = True
            for j in st:
                if j ==' ':
                    if t:x = float(ch);ch="";t=False
                    else:y = float(ch);break
                ch+=j
            return x, y
        except Exception as e: print(e);raise e
        raise Exception()

    def ReadOffset(self, st, i):
        try:
            s = float(st)
            self.recv[f'recv{i}']['offset'] = s
            return 0
        except Exception as e: print(e);return 1
        return 1

    def addRecvPos(self, mac:str, pos:tuple[2], time = 'const'):#add receiver data
        if time == 'const':ntime=time
        else:ntime = round(time*16)
        if mac not in self.recv:
            self.recv[mac] = {ntime:pos}
        else:
            if ntime not in self.recv[mac]:
                self.recv[mac][ntime] = pos
            elif ntime=='const':
                self.recv[mac]['const'] = pos
                self.recv[mac]['offset'] = 0

    def rssToDist(self, rssi):
        e = rssi/-5
        r = rssi%5
        ar = math.pow(10,(-40-rssi)/40)
        return ar

    def readDataTime(self, time, datas):#get position of all nodes at given time
        nodeout = {}
        for i in datas:
            nodeout[i]={}
            for t in datas[i]:
                nodeout[i][t]={}
                for r in datas[i][t]:
                    nodeout[i][t][r]=self.rssToDist(datas[i][t][r])
        return nodeout

    def findNames(self, st=""):
        if st=="":
            return self.nodes.keys(), ['none']
        try:
            st +=' '
            ch = ""
            t = []
            for j in st:
                if j ==' ':
                    t.append(ch);ch=""
                ch+=j
            return t, []
        except Exception as e: print(e);return 1
        return 1

    def readDataNodes(self, names):#get position of given node at all times
        datas = {}
        #print(names)
        for i in names:
            if i in self.nodes:
                datas[i] = copy.deepcopy(self.nodes[i])
        return datas

    def confirmFiles(self, ct):
        fnames = []
        if sys.platform in ["linux","linux2"]:
            directory = os.fsencode(os.getcwd())
            for file in os.listdir(directory):
                fnames.append(os.fsdecode(file))
            for i in range(ct):
                if f'data{i}.bin' not in fnames:
                    return 1
        else: return 1
        return 0

    def readFiles(self, ct):#read all files into memory
        for i in range(ct):
            recv = f"recv{i}"
            with open(f"data{i}.bin", "r") as f:
                time = 0
                while True:
                    c = f.read(1)
                    if not c: break
                    try:
                        if c == 'Z':
                            c = f.read(1)
                            if c == 'Y':#data
                                nm = f.read(12)
                                c = f.read(8)
                                rs = int.from_bytes(bytearray(int(c, 16).to_bytes(int(len(c)/2), 'little')), signed= True)
                                c = f.read(4)
                                times = int.from_bytes(bytearray(int(c, 16).to_bytes(int(len(c)/2), 'little')), signed= False)
                                tm = times/16 +time
                                self.addData(nm, recv, tm, rs)
                            if c == 'X':#time
                                c = f.read(4)
                                time = int.from_bytes(bytearray(int(c, 16).to_bytes(int(len(c)/2), 'little')), signed= True)
                                time = time & 0xFF00
                    except Exception as e:print(e);continue

    def findpos(self, arr, at, bt, const=True):#find the position of node using rssi and positions of receivers
        #[i,t,r,d])

        ret = []
        for i in arr:
            for t in arr[i]:
                a = []
                for r in arr[i][t]:
                    if const:c = self.recv[r]['const']
                    else:c = [0,0]
                    a.append([arr[i][t][r], c]) #[dist,[x,y]]

                if len(a)>=3:
                    if at<=(t/16)<=bt:
                        ret.append([t, minimize(least_squares, [0,0], args=(a))])

        br = {}
        for i in ret:
            tm = int(i[0]//8)
            if tm in br.keys():
                br[tm].append(i[1])
            else:
                br[tm]=[i[1]]
        out = []
        for i in br.keys():
                x = 0
                y = 0
                mxy = 0
                for j in br[i]:
                    x += j.x[0]
                    y += j.x[1]
                    mxy += 1
                    if mxy>2:
                        out.append([x/mxy, y/mxy, mxy/len(br[i])])
                        x = 0
                        y = 0
                        mxy = 0
        return out#a.x[0], a.x[1], a.fun

########Processing data
def least_squares(node, pos):#least square error calcualtion
        err = 0
        for i in pos:
            res_dist = np.sqrt((node[0] -i[1][0])**2 + (node[1]- i[1][1])**2)
            err += (res_dist-i[0])**2
        return err


##GUI
class maketk:
    def __init__(self):
        self.rdm = []
        for i in range(100):
            self.rdm.append(random.random())
        self.img = mpimg.imread('background.png')
        self.root = tkinter.Tk()
        self.root.title("Results")
        self.loader = Loader()
        self.nodedata = nodeData()

        # Figure
        fig = Figure(figsize=(10, 10), dpi=100)
        self.pl = fig.add_subplot()
        self.ax = fig.axes[0]
        self.ax.imshow(self.img, aspect='auto')
        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

        # GUI frames
        frame1 = tkinter.Frame(self.root)
        frame1.pack(side=tkinter.TOP, pady=5)
        frame2 = tkinter.Frame(self.root)
        frame2.pack(side=tkinter.TOP, pady=5)
        frame3 = tkinter.Frame(self.root)
        frame3.pack(side=tkinter.TOP, pady=5)
        frame4 = tkinter.Frame(self.root)
        frame4.pack(side=tkinter.TOP, pady=5)

        #buttons, etc

        tkinter.Button(frame3, text="Download File", command=self.getuart).pack(side=tkinter.LEFT, padx=5)
        #tkinter.Button(frame3, text="Load Files", command=self.loadfiles).pack(side=tkinter.LEFT, padx=5)
        #tkinter.Button(frame3, text="Enter Tracker Coordinates", command=self.getcoord).pack(side=tkinter.LEFT, padx=5)
        self.text = tkinter.Entry(frame3, width=30)
        #self.text.pack(side=tkinter.LEFT, padx=5)
        #tkinter.Button(frame3, text="Read Entry", command=self.gettext).pack(side=tkinter.LEFT, padx=5)
        tkinter.Button(frame3, text="Exit", command=self.exitbutton).pack(side=tkinter.LEFT, padx=5)

        tkinter.Label(frame1, width=20, text = "Tracker Time Offset:").pack(side=tkinter.LEFT, padx=5)
        self.t1 = tkinter.Entry(frame1, width=10)
        self.t1.pack(side=tkinter.LEFT, padx=5)
        self.t2 = tkinter.Entry(frame1, width=10)
        self.t2.pack(side=tkinter.LEFT, padx=5)
        self.t3 = tkinter.Entry(frame1, width=10)
        self.t3.pack(side=tkinter.LEFT, padx=5)
        self.t4 = tkinter.Entry(frame1, width=10)
        self.t4.pack(side=tkinter.LEFT, padx=5)

        tkinter.Label(frame1, width=20, text = "Search Time, space sep:").pack(side=tkinter.LEFT, padx=5)
        self.s1 = tkinter.Entry(frame1, width=15)
        self.s1.pack(side=tkinter.LEFT, padx=5)

        tkinter.Label(frame2, width=23, text = "Tracker Position, space sep:").pack(side=tkinter.LEFT, padx=5)
        self.p1 = tkinter.Entry(frame2, width=10)
        self.p1.pack(side=tkinter.LEFT, padx=5)
        self.p2 = tkinter.Entry(frame2, width=10)
        self.p2.pack(side=tkinter.LEFT, padx=5)
        self.p3 = tkinter.Entry(frame2, width=10)
        self.p3.pack(side=tkinter.LEFT, padx=5)
        self.p4 = tkinter.Entry(frame2, width=10)
        self.p4.pack(side=tkinter.LEFT, padx=5)

        tkinter.Label(frame2, width=17, text = "MAC filter, space sep:").pack(side=tkinter.LEFT, padx=5)
        self.g1 = tkinter.Entry(frame2, width=30)
        self.g1.pack(side=tkinter.LEFT, padx=5)
        tkinter.Button(frame2, text="Plot", command=self.plotall).pack(side=tkinter.LEFT, padx=5)

        self.logs = ScrolledText(frame4, height=5, state='disabled', wrap='word')
        self.logs.pack(side=tkinter.LEFT, fill=tkinter.BOTH, padx=5)

    def exitbutton(self):
        self.root.quit()
        self.root.destroy()

    def getuart(self):
        self.log('Attempting download data over UART...')
        x = self.loader.init_uart()
        self.log('Download Failed' if _o(x) else 'Downloaded')

    def loadfiles(self):
        self.log("Enter the amount of receivers, press 'Read Entry' to start loading all files")

    def log(self, inp):
        clock = datetime.datetime.now().strftime('%H:%M:%S')
        msg = f"[{clock}] {inp}\n"
        self.logs.configure(state='normal')
        self.logs.insert(tkinter.END, msg)
        self.logs.see(tkinter.END)
        self.logs.configure(state='disabled')
        self.root.update()

    def gettext(self):
        print(self.t1.get())


    def getcoord(self):
        self.log("Enter coordinates of trackers in order as files downloaded. In meters can be decimal, x first then space then y, then press 'Read Entry'. Press this button to restart.")
        self.recvCtr = 0
        self.inputMode = 0

    def plotall(self):
        self.nodedata = nodeData()

        x = self.nodedata.confirmFiles(4)
        if _o(x):
                    self.log("File load failed")
        else:
                    self.nodedata.readFiles(4)
                    self.log("Files loaded")
        self.log("Attempting plot, this will take a while...")

        for i, n in enumerate([self.p1, self.p2, self.p3, self.p4]):
            inputs = n.get()
            x = self.nodedata.ReadRecv(inputs, i)
            if _o(x):
                self.log("Invalid input, enter only numbers, spaces and dots: "+ inputs)
            else:
                self.log("Read recv "+str(i)+" pos: "+ inputs)

        for i, n in enumerate([self.t1, self.t2, self.t3, self.t4]):
            inputs = n.get()
            x = self.nodedata.ReadOffset(inputs, i)
            if _o(x):
                self.log("Invalid input, enter only numbers, spaces and dots: "+ inputs)
            else:
                self.log("Read recv"+str(i)+" offset: "+ inputs)

        inputs = self.g1.get()
        try:
            st, xxf = self.nodedata.findNames(inputs)
            ndata = self.nodedata.readDataNodes(list(st))
            self.log("Read mac Addresses ")
        except Exception as e:
            print(e)
            self.log("Invalid input, enter only numbers, spaces and dots: "+ inputs)

        inputs = self.s1.get()
        try:
            a, b = self.nodedata.ReadTimes(inputs)
            a *=16;b*=16
            self.log(f"Read time frame: {a} - {b}")
        except Exception as e:
            print(e)
            self.log("Invalid input, enter only numbers, spaces and dots: "+ inputs)
        self.ax.clear()
        self.ax.imshow(self.img, aspect='auto')
        self.testdraw([], a, b, list(st), xxf)
        self.canvas.draw()
        self.log("plotted")
        return


        #print(self.nodedata.nodes)
        try:
            nodes = self.nodedata.readDataTime([a,b], ndata)  #[i,tt,r,d])
            pos = self.nodedata.findpos(nodes, a, b)
            self.log("Got nodes")
            self.ax.clear()
            self.ax.imshow(self.img, aspect='auto')
            self.testdraw(pos)
            self.canvas.draw()
            self.log("plotted")
        except Exception as e:
            print(e)
            self.log("Error getting nodes")

        #print(self.nodedata.nodes)
    def testdraw(self, pos, c =None, hideRcv=False, rcvConst=True, cc=False):
        for i in pos:
            a = (i[0], i[1], i[2])
            rn = range(np.shape(a)[0])
            self.pl.scatter(a[0]*IMAGESCALE, a[1]*IMAGESCALE, s=a[2]*ERROR_SCALE, color='red', cmap='viridis', alpha=0.3)#c = cc if cc else rn, cmap='viridis'
        if not hideRcv:
                for i in self.nodedata.recv:
                    self.pl.scatter(self.nodedata.recv[i]['const'][0]*IMAGESCALE, self.nodedata.recv[i]['const'][1]*IMAGESCALE, s=0.1*ERROR_SCALE, c = 'black', alpha=1)


if __name__=="__main__":
    gui = maketk()
    gui.root.mainloop()
