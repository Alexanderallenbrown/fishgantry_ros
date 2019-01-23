#This script uses matplotlib and numpy to plot servo data with feedback from Arduino

import serial
from matplotlib.pyplot import *
from numpy import *
import time

#create a figure for us to see our data on. MATLAB-like syntax.
fig,ax = subplots(1,1)
#like MATLAB hold on:
ax.hold(True)
#setting the y limits. Your mileage may vary depending on what data you're plotting
ax.set_ylim(-10,190)
#This initializes the figure. Slightly different than MATLAB.
fig.canvas.draw()

#each "plot" object in matplotlib is a line. here, we have one for the reference input, one for the feedback angle.
plt1 = ax.plot(0,0,'r')[0]
plt2 = ax.plot(0,0,'k')[0]
#set up legends and labels. We do this before we start looping to save time.
legend(['command','feedback'],loc=3)
xlabel('Time (s)')
ylabel('Servo Angle (deg)')


############ NOW WE OPEN A FILE. WE WILL STORE THE RAW ARDUINO DATA TO THIS FILE FOR POST-PROCESSING IN MATLAB/ETC.#############
f = open('output.txt','w')
############ NOW WE OPEN A SERIAL PORT SO WE CAN TALK TO THE ARDUINO. ########################
ser = serial.Serial('/dev/tty.usbmodem1421',115200)


#Now, we must take all of the variables we'd like to plot (in this case, time vs. ref, time vs. feedback), and make variables for them.
#These will only contain the "window" of data we'll plot... the last n points. you can modify to plot other stuff too.
#we need to set a buffer equal to the number of points we want to store
buffsize = 600
#initialize a time vector
t = []
#initialize a reference input vector
r = []
#initialize a feedback vector
y = []

#we will need this conversion because the feedback is in counts, and the reference is in degrees. YMMV
conv = 100.0/(300)
#we will need this offset to make the plots compare well. again, these are application specific. See line 78
offset = -50

#Depending on how fast our computer is, we will need to wait before updating the plot. However, we will STILL
#STORE ALL of the datapoints... we just won't "blit" the plot (update it) until enough time has passed that the computer can keep up.
plot_delay = 0.1 #seconds

#we'll want an old time value
oldtime = time.time()

#as long as the Arduino spits data and we haven't closed the program, do:
while 1:
    #read a line (up to \r\n) from the serial port. This is a cool feature that's a bit more annoying in MATLAB.
    line = ser.readline()
    #write this exact line to a file before we do ANYTHING else.
    f.write(line)#write to file

    #if we actually have data (sometimes the computer outruns the ARduino)
    if len(line)>0:
        #the strip command basically strips the \r\n characters from the end. Remember, it read a string, so the data are still just text.
        #the variable line will look something like '1.234,5.0,4.9\r\n'
        stripline = line.strip()
        #now we split the text, which means variable stripline might look like '1.234,5.0,4.9' and splitline would be a list (array) 
        # ['1.234','5.0','4.9'] so each number is separate, but it's still a string. We'll take care of that shortly. 
        splitline = stripline.split(',')
        #if we have enough numbers and not a weird partial line:
        if len(splitline)>=3:
            
            #if we have fewer points than the buffer size (we haven't been running for long):
            if len(t)<buffsize:
                #this appends our newest values to our variables of interest.
                t.append(float(splitline[0]))
                r.append(float(splitline[1]))
                y.append(float(splitline[2])*conv+offset)#this is where that conversion comes in...
            else: #this means that the buffer needs to lose the oldest value, and gain the newest value.
                #make t equal to the second oldest value to the second newest value
                t = t[1:]
                #add the newest value on to the end, maintaining a vector of buffsize.
                t.append(float(splitline[0]))
                y = y[1:]
                y.append(float(splitline[2])*conv+offset)
                r = r[1:]
                r.append(float(splitline[1]))
                
            
            #now, we only update plot every now and then.... so check how long it's been since we updated!
            if time.time()-oldtime>plot_delay:#if enough time has passed
                #set the old time. Maybe this isn't needed?
                oldtime = time.time()
                #set our X limits of the plot to only look at the last 5 seconds of data TODO make 5 a variable!!
                ax.set_xlim(t[-1]-5,t[-1])
                #this sets the line plt1 data to be our updated t and r vectors.
                plt1.set_data(t,r)
                #same as above.
                plt2.set_data(t,y)
                #the draw command is last, and tells matplotlib to update the figure!!
                fig.canvas.draw()
                pause(.001)#must have a small pause here or nothing will work. Pause is a matplotlib.pyplot function.
                
