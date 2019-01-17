#!/usr/bin/env python
""" 
This node takes a tailbeat frequency, amplitude, and bias, simulates an Artemis SS-style single-hinge fish robot, and publishes
relevant transforms, pose, twist, and marker messages.

Alexander Brown, Ph.D. June 2017
"""
import roslib
import rospy
from numpy import *
roslib.load_manifest('fishgantry_ros')
from fishgantry_ros.msg import FishCommandMsg
import tf
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import Joy
from nav_msgs.msg import *
import rigidfish #this is my rigid fish class

class PosePublisher():
    def __init__(self):
        #initialize the ros node
        rospy.init_node('rigidfish_sim')

        #class-owned variables that are NOT ROS types
        #TODO many of these should be ros parameters...
        self.fishsim_x = array([0,0,0,0,0,0],dtype=float64)
        self.tailsim_x = array([0,0],dtype=float64)
        self.fishquat = Quaternion()#a quaternion for fish orientation
        self.tailquat = Quaternion()#a quaternion for tail orientation relative to fish
        self.tailref = 0.
        self.dT = 0.02#this is how fast the simulation loop will run.
        self.timenow = rospy.Time.now()
        self.oldtime = self.timenow
        #TODO make these ros params for SURE. and also on the teleop node for real robot.
        #these are in DEGREES
        self.biasmax = 45.
        self.biasmin = -45.
        self.freqmax = 25.
        self.freqmin = 0.
        self.ampmin = 0.
        self.ampmax = 45.
        self.enable=0
        self.freq = 0
        self.bias = 0
        self.amp = 0
        self.heightgoal=0


        #simulation objects from the rigidfish module
        self.tailcontroller = rigidfish.TailCommandGenerator()
        self.tailsim = rigidfish.TailServo()
        self.fishsim = rigidfish.RigidFish()
        self.tiltgoal = 0
        self.heightgoal = 0

        #publishers
        self.posepub = rospy.Publisher('/fishgantry/commandpose',PoseStamped,queue_size=1)
        self.tailposepub = rospy.Publisher('/fishgantry/tailpose',PoseStamped,queue_size=1)
        self.fishmarkerpub = rospy.Publisher('/fishmarker',Marker,queue_size=1)
        self.tailmarkerpub = rospy.Publisher('/tailmarker',Marker,queue_size=1)

        #create simulation loop
        rospy.Timer(rospy.Duration(self.dT),self.simtimercallback,oneshot=False)
        #create a slow(er) loop to publish markers for RVIZ
        rospy.Timer(rospy.Duration(0.1),self.markertimercallback,oneshot=False)


        #subscribe to joystick TODO add modes for autonomous operation
        self.joy_sub = rospy.Subscriber('/fishgantry/simfish_command',FishCommandMsg,self.commandcallback)

    def simtimercallback(self,data):
        #take care of updating loop rate, etc.
        self.timenow = rospy.Time.now()
        #update dt. TODO to_sec method??
        self.oltime = self.timenow

        #update tail command
        self.tailcontroller.update(self.enable,self.freq,self.amp,self.bias,self.dT)#TODO make the timing more precise... get the ACTUAL time
        #update the tail states
        self.tailsim.EulerUpdateStates(self.tailcontroller.theta_ref,self.dT)
        #now update the fish states
        self.fishsim.EulerUpdateStates(self.tailsim.x[0],self.tailsim.xdot[1],self.dT,self.tiltgoal,self.heightgoal)
        
        #now publish the appropriate things: Poses for fish and tail, transforms for fish and tail, and markers for fish and tail

        #publish transform for fish and one for tail
        br = tf.TransformBroadcaster()
        fishquat = tf.transformations.quaternion_from_euler(0,0,self.fishsim.x[5])#TODO add static pitch angle of fish (also add to sim?)
        br.sendTransform((self.fishsim.x[3],self.fishsim.x[4],self.heightgoal),fishquat,self.timenow,'/fishsim','/world')

        tailquat = tf.transformations.quaternion_from_euler(0,0,self.tailsim.x[0])
        br.sendTransform((-3.25*.0254,0,0),tailquat,self.timenow,'/tailsim','/fishsim')

        #prepare messages for fish and tail position and velocity.
        fishpose_msg = PoseStamped()
        fishpose_msg.header.stamp = self.timenow
        fishpose_msg.pose.position.x = self.fishsim.x[3]
        fishpose_msg.pose.position.y = self.fishsim.x[4]
        fishpose_msg.pose.position.z = self.fishsim.height
        fishpose_msg.pose.orientation.x = self.fishsim.tilt
        fishpose_msg.pose.orientation.y = 0
        fishpose_msg.pose.orientation.z = self.fishsim.x[5]
        fishpose_msg.header.frame_id='/world'

        tailpose_msg = PoseStamped()
        tailpose_msg.header.stamp = self.timenow
        tailpose_msg.pose.position.x = 0
        tailpose_msg.pose.position.y = 0
        tailpose_msg.pose.position.z = 0
        tailpose_msg.pose.orientation.x = 0
        tailpose_msg.pose.orientation.y = 0
        tailpose_msg.pose.orientation.z = self.tailsim.x[0]
        tailpose_msg.header.frame_id='/fishsim'

        # fishtwist_msg = TwistStamped()
        # fishtwist_msg.header.stamp = self.timenow
        # fishtwise_msg.twist.linear

        self.tailposepub.publish(tailpose_msg)
        self.posepub.publish(fishpose_msg)



    def markertimercallback(self,data):
        #publish a marker representing the fish body position
        fishmarker = Marker()
        fishmarker.header.frame_id='/fishsim'
        fishmarker.header.stamp = self.timenow
        fishmarker.type = fishmarker.MESH_RESOURCE
        fishmarker.mesh_resource = 'package://servofish_ros/meshes/fishbody.dae'
        fishmarker.mesh_use_embedded_materials = True
        fishmarker.action = fishmarker.MODIFY
        fishmarker.scale.x = 1
        fishmarker.scale.y = 1
        fishmarker.scale.z = 1
        tempquat = tf.transformations.quaternion_from_euler(pi/2,0,pi/2)#this is RELATIVE TO FISH ORIENTATION IN TF (does the mesh have a rotation?)
        fishmarker.pose.orientation.w = tempquat[3]
        fishmarker.pose.orientation.x = tempquat[0]
        fishmarker.pose.orientation.y = tempquat[1]
        fishmarker.pose.orientation.z = tempquat[2]
        fishmarker.pose.position.x = 0
        fishmarker.pose.position.y = 0
        fishmarker.pose.position.z = 0
        fishmarker.color.r = .5
        fishmarker.color.g = .8
        fishmarker.color.b = .5
        fishmarker.color.a = 1.0#transparency

        #publish a marker representing the fish tail
        tailmarker = Marker()
        tailmarker.header.frame_id='/tailsim'
        tailmarker.header.stamp = self.timenow
        tailmarker.type = tailmarker.MESH_RESOURCE
        tailmarker.mesh_resource = 'package://fishgantry_ros/meshes/fishtail.dae'
        tailmarker.mesh_use_embedded_materials = True
        tailmarker.action = tailmarker.MODIFY
        tailmarker.scale.x = 1
        tailmarker.scale.y = 1
        tailmarker.scale.z = 1
        tempquat = tf.transformations.quaternion_from_euler(pi/2,0,pi/2)#this is RELATIVE TO tail ORIENTATION IN TF (does the mesh have a rotation?)
        tailmarker.pose.orientation.w = tempquat[3]
        tailmarker.pose.orientation.x = tempquat[0]
        tailmarker.pose.orientation.y = tempquat[1]
        tailmarker.pose.orientation.z = tempquat[2]
        tailmarker.pose.position.x = 3.25*.0254
        tailmarker.pose.position.y = 0
        tailmarker.pose.position.z = 0
        tailmarker.color.r = .5
        tailmarker.color.g = .5
        tailmarker.color.b = .8
        tailmarker.color.a = 1.0#transparency

        self.fishmarkerpub.publish(fishmarker)
        self.tailmarkerpub.publish(tailmarker)

    def commandcallback(self,data):
        self.freq = data.tail_freq
        self.bias = data.tail_bias
        self.amp = data.tail_amp
        self.enable = data.tail_enable
        self.heightgoal = data.height_goal
        self.tiltgoal = data.tilt_goal
        self.tail_enable = data.tail_enable
        # if data.buttons[6]==1:
        #     self.tailcontroller.update(1,freqcommand,ampcommand,biascommand)
        # else:
        #     self.tailcontroller.update(0,freqcommand,ampcommand,biascommand)

def main(args):
    posepub = PosePublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__ == '__main__':
    main(sys.argv)