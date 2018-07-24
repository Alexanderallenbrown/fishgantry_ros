#!/usr/bin/env python

import roslib; roslib.load_manifest('fishgantry_ros')
import rospy
# from tetherfish_ros.msg import TetherFishMsg
import tf
import serial
import sys
import datetime
import rospkg
import nav_msgs

class FishGantry():


   def __init__(self):
      self.port = rospy.get_param('~port','/dev/ttyUSB0')
      self.baud = rospy.get_param('~baud',115200)
      self.ser = serial.Serial(self.port, self.baud,timeout=1) #this initializes the serial object
      self.pub= rospy.Publisher("fishgantry/robotpose",PoseStamped)
      self.goalpose_sub = rospy.Subscriber("/gantry/commandpose",PoseStamped,self.commandCallback)
      
      #initialize a command position
      self.command = PoseStamped()
      self.command.header.stamp = rospy.Time.now()
      self.pose.position.x = 0
      self.pose.position.y = 0
      self.pose.position.z = 0

      #main loop runs on a timer, which ensures that we get timely updates from the gantry arduino
      rospy.Timer(rospy.Duration(.005),self.loop,oneshot=False) #timer callback (math) allows filter to run at constant time
      #subscribers (inputs)
      #construct the file name for our text output file
      rospack = rospkg.RosPack()
      # get the file path for rospy_tutorials
      self.package_path=rospack.get_path('fishgantry_ros')

   def loop(self,event):
      line = self.ser.readline()
      print line
      linestrip = line.strip('\r\n')
      linesplit = line.split()
      if len(linesplit)>=3:
        #print shotslast, arduinonumshots,shotslast==(arduinonumshots+1)
        
        try:

            msg = TetherFishMsg()
            msg.header.stamp = rospy.Time.now()
            msg.robotpos = float(linesplit[0])
            msg.robotangle = float(linesplit[1])
            msg.arduino_time = float(linesplit[2])
            #msg.tailfreq= float(linesplit[3])
            #msg.tailamp = float(linesplit[4])
            #msg.tailangle = float(linesplit[5])
            print "publishing"
            self.pub.publish(msg)
            

        except:
            print "OOPS! BAD LINE"
            #ef.write("problem  with serial line"+"\r\n")

      
#main function
def main(args):
  rospy.init_node('gantry_node', anonymous=True)
  encreader = ArduinoTarget()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 