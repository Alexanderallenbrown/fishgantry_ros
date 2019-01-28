# from matplotlib.backends.backend_tkagg import (
#     FigureCanvasTkAgg, NavigationToolbar2Tk)
# # Implement the default Matplotlib key bindings.
# from matplotlib.backend_bases import key_press_handler
# from matplotlib.figure import Figure
# from numpy import *
# from matplotlib.pyplot import *
# #import tkinter
# from Tkinter import *

# import os

import serial
import time
# import sys
from Tkinter import *
# import tkinter

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

from numpy import *


class EllipticalPath():
    def __init__(self,a=.1,b=.1,U=.05):
        self.a,self.b,self.U = a,b,U
        self.theta = arange(0,2*pi,.01) #range of thetas
        self.x = a*cos(self.theta)+a
        self.y = b*sin(self.theta)+b
        self.S = zeros(len(self.y))
        self.yaw = zeros(len(self.y))
        roll=0
        self.yaw[0] = arctan2(self.y[1]-self.y[0],self.x[1]-self.x[0])
        for ind in range(1,len(self.S)):
            delta_S = sqrt((self.x[ind]-self.x[ind-1])**2+(self.y[ind]-self.y[ind-1])**2)
            self.S[ind]=self.S[ind-1]+delta_S
            self.yaw[ind] = arctan2(self.y[ind]-self.y[ind-1],self.x[ind]-self.x[ind-1])
            if(abs(self.yaw[ind]-self.yaw[ind-1])>=pi):
                roll=1
            self.yaw[ind] = self.yaw[ind]+roll*2*pi

        for k in range(0,len(self.yaw)):
            print self.yaw[k]
        self.maxS = self.S[-1]
        sind = where(self.theta>(pi))[0][0]
        self.Snow = self.S[sind]
        self.laps = 0

        self.xnow,self.ynow,self.yawnow = 0,0,0

    def update(self,dt,U):
        self.U = U
        if(self.Snow)>self.maxS:
            self.Snow-=self.maxS
            self.laps+=1
        self.Snow += dt*self.U
        self.xnow = interp(self.Snow,self.S,self.x)
        self.ynow = interp(self.Snow,self.S,self.y)
        self.yawnow = interp(self.Snow,self.S,self.yaw)
        #print self.Snow,self.maxS
        return self.xnow, self.ynow,self.yawnow

    def updateGeometry(self,a,b,U):
        self.a,self.b,self.U = a,b,U
        self.theta = arange(0,2*pi,.01) #range of thetas
        self.x = a*cos(self.theta)+a
        self.y = b*sin(self.theta)+b
        self.S = zeros(len(self.y))
        roll=0
        self.yaw[0] = arctan2(self.y[1]-self.y[0],self.x[1]-self.x[0])
        for ind in range(1,len(self.S)):
            delta_S = sqrt((self.x[ind]-self.x[ind-1])**2+(self.y[ind]-self.y[ind-1])**2)
            self.S[ind]=self.S[ind-1]+delta_S
            self.yaw[ind] = arctan2(self.y[ind]-self.y[ind-1],self.x[ind]-self.x[ind-1])
            if(abs(self.yaw[ind]-self.yaw[ind-1])>=pi):
                roll=1
            self.yaw[ind] = self.yaw[ind]+roll*2*pi

        self.maxS = self.S[-1]
        sind = where(self.theta>(pi))[0][0]
        self.Snow = self.S[sind]

        self.xnow,self.ynow = 0,0


class Window():
    def __init__(self, master=None):
        #Frame.__init__(self, master)    
        self.running = False     
        self.delay = 10 #milliseconds
        self.refreshdelay = 100
        self.tnow = time.time()
        self.starttime =self.tnow
        self.sendHome = False
        self.disable = 0
        self.olddisable = 0

        self.master = master
        #numpy stuff
        self.fig = Figure(figsize=(5,4),dpi=100)
        self.t = arange(0,3,.01)
        self.ax = self.fig.add_subplot(111)
        self.commandline,=self.ax.plot(self.t,2*sin(2*pi*self.t),'r')
        self.feedbackline,=self.ax.plot(self.t,2*sin(2*pi*self.t+pi/6),'k')
        self.ax.legend(['command','feedback'],loc=1)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Output')

        #serial stuff
        self.port = ''
        self.baud = None
        self.ser = None

        #buffer size
        self.bufsize = 500#how many points to store

        #actual variables for holding command values. these will become lists
        self.c1 = []
        self.c2 = []
        self.c3 = []
        self.c4 = []
        self.c5 = []
        self.c6 = []
        self.f1 = []
        self.f2 = []
        self.f3 = []
        self.f4 = []
        self.f5 = []
        self.tvec = []

        self.xmax = 1
        self.ymax=1
        self.zmax=1
        self.pmax=1
        self.amax=1

        self.plotaxis = 1
        self.sliderscale = 1000

        #stuff for elliptical patth
        self.U = .05
        self.a = .4
        self.b = .1
        self.path = EllipticalPath(self.a,self.b,self.U)
        self.pathActive = False
        self.pathWasActive = False

        #initialize the window
        self.init_window()
    def init_window(self):
        self.master.title("GANTRY CONTROL")
        # allowing the widget to take the full space of the root window
        #self.master.pack(fill=BOTH, expand=1)
        self.lside = Frame(self.master)
        #self.rrside = Frame(self.master)
        self.mside = Frame(self.master)
        self.rside = Frame(self.master)
        self.lside.pack(side="left")
        #self.rrside.pack(side="right")
        self.mside.pack(side="left")
        self.rside.pack(side="bottom")

        #fake button for running timed loop
        self.loopbutton = Button(self.master)
        self.refreshbutton = Button(self.master)
        self.pathbutton = Button(self.master)

        #label and box for serial port
        Tport=StringVar()
        Tport.set("Serial Port")
        Lport=Label(self.master, textvariable=Tport, height=1)
        Lport.pack(in_=self.lside,side="top")
        SVport = StringVar()
        SVport.set("/dev/cu.usbmodem1421")
        self.Eport =Entry(self.master,textvariable=SVport,width=20)
        self.Eport.pack(in_=self.lside,side="top")
        #label and box for baud rate
        Tbaud=StringVar()
        Tbaud.set("Baud Rate")
        Lbaud=Label(self.master, textvariable=Tbaud, height=1)
        Lbaud.pack(in_=self.lside,side="top")
        SVbaud = StringVar()
        SVbaud.set("115200")
        self.Ebaud =Entry(self.master,textvariable=SVbaud,width=10)
        self.Ebaud.pack(in_=self.lside,side="top")
        #button for opening port
        seropen = Button(self.master, text="Open Port",command=self.startserial)
        seropen.pack(in_=self.lside,side="top")
        serclose = Button(self.master, text="Close Port",command=self.endserial)
        serclose.pack(in_=self.lside,side="top")

        #now sliders for each axis (6 total)
        Tx=StringVar()
        Tx.set("x command")
        self.sx = Scale(self.master,from_=0,to=self.sliderscale,orient=HORIZONTAL,length=200)
        self.sx.pack(in_=self.lside,side="top")
        Lx=Label(self.master, textvariable=Tx, height=1)
        Lx.pack(in_=self.lside,side="top")

        Ty=StringVar()
        Ty.set("y command")
        self.sy = Scale(self.master,from_=0,to=self.sliderscale,orient=HORIZONTAL,length=200)
        self.sy.pack(in_=self.lside,side="top")
        Ly=Label(self.master, textvariable=Ty, height=1)
        Ly.pack(in_=self.lside,side="top")

        Tz=StringVar()
        Tz.set("z command")
        self.sz = Scale(self.master,from_=-self.sliderscale,to=0,orient=HORIZONTAL,length=200)
        self.sz.pack(in_=self.lside,side="top")
        Lz=Label(self.master, textvariable=Tz, height=1)
        Lz.pack(in_=self.lside,side="top")

        Tp=StringVar()
        Tp.set("pitch command")
        self.sp = Scale(self.master,from_=0,to=self.sliderscale,orient=HORIZONTAL,length=200)
        self.sp.pack(in_=self.lside,side="top")
        Lp=Label(self.master, textvariable=Tp, height=1)
        Lp.pack(in_=self.lside,side="top")

        Ta=StringVar()
        Ta.set("yaw command")
        self.sa = Scale(self.master,from_=0,to=self.sliderscale,orient=HORIZONTAL,length=200)
        self.sa.pack(in_=self.lside,side="top")
        La=Label(self.master, textvariable=Ta, height=1)
        La.pack(in_=self.lside,side="top")

        Tt=StringVar()
        Tt.set("tail command")
        self.st = Scale(self.master,from_=-90,to=90,orient=HORIZONTAL,length=200)
        self.st.set(0)
        self.st.pack(in_=self.lside,side="top")
        Lt=Label(self.master, textvariable=Tt, height=1)
        Lt.pack(in_=self.lside,side="top")
        
        #create a home button
        # SV = StringVar()
        # SV.set("Home ALL")
        # L = Label(self.master,textvariable=SV,height=1)
        # L.pack(in_=self.mside,side="top")
        # SV = StringVar()
        self.Hbutton = Button(master=self.master, text="Home", command=self.sethome)
        self.Hbutton.pack(in_=self.mside,side="top")

        self.enbox = Checkbutton(self.master, text="Disable", variable=self.disable)
        self.enbox.pack(in_=self.mside,side="top")

        #make button for activating elliptical path
        self.Pbutton = Button(master=self.master, text="Enable Elliptical Path", command=self.setpath)
        self.Pbutton.pack(in_=self.mside,side="top")

        Ta=StringVar()
        Ta.set("Path Speed (mm/s)")
        self.sU = Scale(self.master,from_=0,to=300,orient=HORIZONTAL,length=200)
        self.sU.pack(in_=self.mside,side="top")
        La=Label(self.master, textvariable=Ta, height=1)
        La.pack(in_=self.lside,side="top")

        #create the dropdown menu for axes:
        SV = StringVar()
        SV.set("Axis To Plot")
        L = Label(self.master,textvariable=SV,height=1)
        L.pack(in_=self.mside,side="top")
        self.axisstring = StringVar()
        self.axisstring.set("x axis")
        self.om = OptionMenu(self.master, self.axisstring, "x axis", "y axis", "z axis","tilt axis","yaw axis","xy plan view")
        self.om.pack(in_=self.mside,side="top")
        
        #set up the canvas for the figure
        self.canvas = FigureCanvasTkAgg(self.fig,master=self.mside)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(in_=self.mside,side="top")
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.mside)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(in_=self.mside,side="top")
        self.canvas.mpl_connect("key_press_event", self.on_key_press)
        self.qbutton = Button(master=self.master, text="Quit", command=self._quit)
        self.qbutton.pack(in_=self.mside,side="bottom")

        #now deal with the utilities: drop-down for which axis to look at, refresh rate, buffer size, etc.

        s=StringVar()
        s.set("x max")
        l=Label(self.master, textvariable=s, height=1)
        l.pack(in_=self.mside,side="left")
        SV = StringVar()
        SV.set("0.4")
        self.Exmax =Entry(self.master,textvariable=SV,width=5)
        self.Exmax.pack(in_=self.mside,side="left")

        s=StringVar()
        s.set("y max")
        l=Label(self.master, textvariable=s, height=1)
        l.pack(in_=self.mside,side="left")
        SV = StringVar()
        SV.set("0.15")
        self.Eymax =Entry(self.master,textvariable=SV,width=5)
        self.Eymax.pack(in_=self.mside,side="left")

        s=StringVar()
        s.set("z max")
        l=Label(self.master, textvariable=s, height=1)
        l.pack(in_=self.mside,side="left")
        SV = StringVar()
        SV.set("0.1")
        self.Ezmax =Entry(self.master,textvariable=SV,width=5)
        self.Ezmax.pack(in_=self.mside,side="left")

        s=StringVar()
        s.set("tilt max")
        l=Label(self.master, textvariable=s, height=1)
        l.pack(in_=self.mside,side="left")
        SV = StringVar()
        SV.set("0.5")
        self.Epmax =Entry(self.master,textvariable=SV,width=5)
        self.Epmax.pack(in_=self.mside,side="left")

        s=StringVar()
        s.set("yaw max")
        l=Label(self.master, textvariable=s, height=1)
        l.pack(in_=self.mside,side="left")
        SV = StringVar()
        SV.set("12.6")
        self.Eamax =Entry(self.master,textvariable=SV,width=5)
        self.Eamax.pack(in_=self.mside,side="left")  
        
    
    def setpath(self):
        self.pathActive = not self.pathActive
        #get the a and b for this ellipse, and update the path geometry
        if(self.pathActive):
            self.xmax=float(self.Exmax.get())
            self.ymax=float(self.Eymax.get())
            self.path.updateGeometry(self.xmax/2,self.ymax/2,self.sU.get()/1000.0)
            print("updated path geometry")
        #update the path x and y positions
        if(not self.pathActive):
            self.Pbutton.config(text="Enable Elliptical Path")
        else:
            self.Pbutton.config(text="Disable Elliptical Path")
            self.pathbutton.after(self.delay,self.pathloop)
        



    def pathloop(self):
        #if path is active, update the x and y commands
        x,y,yaw = self.path.update(self.delay/1000.0,self.sU.get()/1000.0)
        #print(x,y,yaw)
        #now set the x and y sliders accordingly
        self.sx.set(x*self.sliderscale/self.xmax)
        self.sy.set(y*self.sliderscale/self.ymax)
        self.sa.set(yaw*self.sliderscale/self.amax)
        #if the path is active, run it again
        if(self.pathActive):
            self.pathbutton.after(self.delay,self.pathloop)


    def sethome(self):
        self.sendHome = True

    def loop(self):
        #do all of the things
        #print "running"
        #first, get the value from each slider
        if(not self.sendHome):
            c1,c2,c3,c4,c5,c6 = float(self.sx.get())*self.xmax/self.sliderscale,float(self.sy.get())*self.ymax/self.sliderscale,float(self.sz.get())*self.zmax/self.sliderscale,float(self.sp.get())*self.pmax/self.sliderscale,(float(self.sa.get()))*self.amax/self.sliderscale+self.path.laps*2*pi,float(self.st.get())
        else:
            c1,c2,c3,c4,c5,c6 = -111.1,-111.1,-111.1,-111.1,-111.1,0
            self.sendHome = False

        if(self.disable==1 and self.olddisable==0):
            c1,c2,c3,c4,c5,c6 = -222.2,-222.2,-222.2,-222.2,-222.2,0
        elif(self.disable==0 and self.olddisable == 1):
            c1,c2,c3,c4,c5,c6 = -333.3,-333.3,-333.3,-333.3,-333.3,0
        self.olddisable = self.disable

        #print c1,c2,c3,c4,c5,c6
        strcom = '!'+"{0:.4f}".format(c1)+","+"{0:.4f}".format(c2)+","+"{0:.4f}".format(c3)+","+"{0:.4f}".format(c4)+","+"{0:.4f}".format(c5)+","+"{0:.4f}".format(c6)+'\r\n'
        # HERE IS WHERE SERIAL GOES
        self.ser.write(strcom)
        line = self.ser.readline()
        #print line
        #if we actually have data (sometimes the computer outruns the ARduino)
        if len(line)>0:
            #print line
            #the strip command basically strips the \r\n characters from the end. Remember, it read a string, so the data are still just text.
            #the variable line will look something like '1.234,5.0,4.9\r\n'
            stripline = line.strip()
            #now we split the text, which means variable stripline might look like '1.234,5.0,4.9' and splitline would be a list (array) 
            # ['1.234','5.0','4.9'] so each number is separate, but it's still a string. We'll take care of that shortly. 
            splitline = stripline.split('\t')
            #if we have enough numbers and not a weird partial line:
            if len(splitline)==6:
                ardt,f1,f2,f3,f4,f5 = float(splitline[0]),float(splitline[1]),float(splitline[2]),float(splitline[3]),float(splitline[4]),float(splitline[5])
                #if we have fewer points than the buffer size (we haven't been running for long):

        #get current wall time
        self.tnow = time.time()-self.starttime
        #now append each of these values to their respective vectors
        if len(self.c1)>self.bufsize: #should be the same as checking them all...
            self.c1 = self.c1[1:]
            self.c2 = self.c2[1:]
            self.c3 = self.c3[1:]
            self.c4 = self.c4[1:]
            self.c5 = self.c5[1:]
            self.c6 = self.c6[1:]
            self.tvec = self.tvec[1:]
            self.f1 = self.f1[1:]
            self.f2 = self.f2[1:]
            self.f3 = self.f3[1:]
            self.f4 = self.f4[1:]
            self.f5 = self.f5[1:]
        self.c1.append(c1)
        self.c2.append(c2)
        self.c3.append(c3)
        self.c4.append(c4)
        self.c5.append(c5)
        self.c6.append(c6)

        self.f1.append(f1)
        self.f2.append(f2)
        self.f3.append(f3)
        self.f4.append(f4)
        self.f5.append(f5)
        
        self.tvec.append(self.tnow)
        if(self.running):
            self.loopbutton.after(self.delay,self.loop)

    def refresh(self):
        #print "refreshing"

        self.plotaxis = self.axisstring.get()
        #print self.plotaxis

        #actually set the data for each line
        if(self.plotaxis=="x axis"):
            self.commandline.set_data(self.tvec,self.c1)
            self.feedbackline.set_data(self.tvec,self.f1)
             #set the axis limits
            self.ax.set_xlim([self.tvec[0],self.tvec[-1]])
            self.ax.set_ylim([0,self.xmax])
            self.ax.set_ylabel('x axis (m)')
            self.ax.set_xlabel("Time (s)")
        elif(self.plotaxis=="y axis"):
            self.commandline.set_data(self.tvec,self.c2)
            self.feedbackline.set_data(self.tvec,self.f2)
            #set the axis limits
            self.ax.set_xlim([self.tvec[0],self.tvec[-1]])
            self.ax.set_ylim([0,self.ymax])
            self.ax.set_ylabel('y axis (m)')
            self.ax.set_xlabel("Time (s)")
        elif(self.plotaxis=="z axis"):
            self.commandline.set_data(self.tvec,self.c3)
            self.feedbackline.set_data(self.tvec,self.f3)
            #set the axis limits
            self.ax.set_xlim([self.tvec[0],self.tvec[-1]])
            self.ax.set_ylim([0,self.zmax])
            self.ax.set_ylabel('z axis (m)')
            self.ax.set_xlabel("Time (s)")
        elif(self.plotaxis=="tilt axis"):
            self.commandline.set_data(self.tvec,self.c4)
            self.feedbackline.set_data(self.tvec,self.f4)
            #set the axis limits
            self.ax.set_xlim([self.tvec[0],self.tvec[-1]])
            self.ax.set_ylim([0,self.pmax])
            self.ax.set_ylabel('tilt axis (rad)')
            self.ax.set_xlabel("Time (s)")
        elif(self.plotaxis=="yaw axis"):
            self.commandline.set_data(self.tvec,self.c5)
            self.feedbackline.set_data(self.tvec,self.f5)
            #set the axis limits
            self.ax.set_xlim([self.tvec[0],self.tvec[-1]])
            self.ax.set_ylim([0,self.amax+self.path.laps*2*pi])
            self.ax.set_ylabel('yaw axis (rad)')
            self.ax.set_xlabel("Time (s)")
        elif(self.plotaxis == "xy plan view"):
            self.commandline.set_data(self.c1,self.c2)
            self.feedbackline.set_data(self.f1,self.f2)
            self.ax.set_xlim([0,self.xmax])
            self.ax.set_ylim([0,self.ymax])
            self.ax.set_aspect('equal')
            self.ax.set_ylabel("y position (m)")
            self.ax.set_xlabel("x position (m)")
       
        self.canvas.draw()
        if(self.running):
            self.refreshbutton.after(self.refreshdelay,self.refresh)


    def on_key_press(self,event):
        print("you pressed {}".format(event.key))
        key_press_handler(event, self.canvas,self. toolbar)
    def _quit(self):
        self.master.quit()     # stops mainloop
        self.master.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate
    def startserial(self):
        print "opening serial..."
        self.running=True
        #pull the port and baud from the input boxes
        self.port = self.Eport.get()
        self.baud = int(self.Ebaud.get())
        self.xmax=float(self.Exmax.get())
        self.ymax=float(self.Eymax.get())
        self.zmax=float(self.Ezmax.get())
        self.pmax=float(self.Epmax.get())
        self.amax=float(self.Eamax.get())
        #actually start the serial port
        self.ser = serial.Serial(self.port,self.baud)
        #when did we start?
        self.starttime = time.time()
        #now start the two timed loops firing
        self.loopbutton.after(self.delay,self.loop)
        self.refreshbutton.after(self.delay,self.refresh)

    def endserial(self):
        print "closing serial..."
        self.ser.close()
        self.running=False



root = Tk()
#size of the window
# root.geometry("600x400")
app = Window(root)
root.mainloop()


# root = tkinter.Tk()
# root.wm_title("Embedding in Tk")

# fig = Figure(figsize=(5, 4), dpi=self.sliderscale)
# t = np.arange(0, 3, .01)
# fig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

# canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
# canvas.draw()
# canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

# toolbar = NavigationToolbar2Tk(canvas, root)
# toolbar.update()
# canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)


# def on_key_press(event):
#     print("you pressed {}".format(event.key))
#     key_press_handler(event, canvas, toolbar)


# canvas.mpl_connect("key_press_event", on_key_press)


# def _quit():
#     root.quit()     # stops mainloop
#     root.destroy()  # this is necessary on Windows to prevent
#                     # Fatal Python Error: PyEval_RestoreThread: NULL tstate


# button = tkinter.Button(master=root, text="Quit", command=_quit)
# button.pack(side=tkinter.BOTTOM)

# tkinter.mainloop()
# # If you put root.destroy() here, it will cause an error if the window is
# # closed with the window manager.