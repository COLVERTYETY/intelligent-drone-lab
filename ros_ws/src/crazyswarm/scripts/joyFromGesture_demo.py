#!/usr/bin/env python

###############
#modified by bubulle for the demo
#changed few things to make it more stable
#topics names are differents

import logging
from pynput import keyboard


import numpy as np
from pycrazyswarm import *
import sys
import signal

import rospy
from std_msgs.msg import String

from collections import deque
import statistics
from statistics import mode#, multimode

import pyttsx3
global engine 
engine = pyttsx3.init()

# Z = 0.3
# sleepRate = 30
velocity_normal = 0.2
velocity_slide = 0.3
takeoff_height = 0.5
msg= ''
followMode=False

signal.signal(signal.SIGINT, signal_handler)

#CTRL C
def signal_handler(signal, frame):
	allcfs.crazyflies[0].land(0.04, 2.5)
	sys.exit(0)

#to have voice saying directions during flight
#currently not used
def speak(engine, text):
    engine.say(text)
    engine.runAndWait()

def print_commands():
	print ('SPIDERMAN to take off!')
        print ('THUMBDOWN to land!')
        print ('UP to move up!')
        print ('DOWN to move down!')
        print ('RIGHT to move right!')
        print ('LEFT to move left!')
        print ('FIST emergency STOP')
        print('PEACE to go on follow mode')
	
def start():
	print(cf)
	print_commands()
	cf.takeoff(targetHeight=self.takeoff_height, duration=3.0)
	print('I took off')
	
	
        print("I began listening")
        handsignal_subscriber = rospy.Subscriber('/hand_signal', String, hand_callback)
        print("I'm listening to hand signal")
	velocity_subscriber = rospy.Subscriber('security_speed', Float64MultiArray, velocity_callback)
	print('listening !')
        
        rospy.spin()

def drone_normal_mode(data):
        if data == 'SPIDERMAN':
                #print("signalmode, spiderman.")
                cf.takeoff(targetHeight=takeoff_height, duration=3.0)

        if data == 'THUMBDOWN':
                #print(".")
                cf.land(0.04, 2.5)

        if data == 'UP':
                #print(".")
                cf.cmdVelocityWorld(np.array([0, 0, velocity_normal]), yawRate=0)
                #rospy.sleep()

        if data == 'DOWN': 
                #print(".")
                cf.cmdVelocityWorld(np.array([0, 0, -velocity_normal]), yawRate=0)
                #rospy.sleep()

        if data == 'RIGHT': 
                #print(".")
                cf.cmdVelocityWorld(np.array([-velocity_normal, 0, 0]), yawRate=0)
                #rospy.sleep()

        if data == 'LEFT': 
                #print(".")
                cf.cmdVelocityWorld(np.array([velocity_normal, 0, 0]), yawRate=0)
                #rospy.sleep()

        if data == '' or data == 'UNKNOWN' or data == 'FIST':
                print(".")
		#test with this line later ?
                #self.cf.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
		
def drone_follow_mode(data):
	if data == 'RIGHT SLIDE' :#start_right
                cf.cmdVelocityWorld(np.array([-velocity_slide, 0, 0]), yawRate=0)
                print("slide right, follow.")

        if data == 'LEFT SLIDE' :#start_left
                print("slide left")
                cf.cmdVelocityWorld(np.array([self.velocity_slide, 0, 0]), yawRate=0)
            
        if data == 'UP SLIDE': #start_up
                print("slide up")       
                cf.cmdVelocityWorld(np.array([0, 0, self.velocity_slide]), yawRate=0)
                
        if data == 'DOWN SLIDE': #start_down
                print("slide down")
                cf.cmdVelocityWorld(np.array([0, 0, -self.velocity_slide]), yawRate=0)

                

def hand_callback(msg):
        print(msg.data)
	
        if msg.data == 'PEACE' and not followMode: #Activate followMODE
		#speak (engine, "Activating Hand following mode.")
		#print ("followMode ACTIVATED")
		followMode=True

        if msg.data == 'INDEX' and followMode: #Activate SignalMode: 
		#speak (engine, "Activating signal mode.")
		#Deactivate followMODE
		#print ("SignalMode ACTIVATED")
		self.followMode=False

        if self.followMode == False :
		drone_normal_mode(msg.data)

        if self.followMode == True:
		print("in FOLLOW mode")
		drone_follow_mode(msg.data)

def velocity_callback(self, msg):
	self.velocity_normal = 0.3 * msg.data
	self.velocity_slide = 0.2 * msg.data


if __name__ == '__main__':
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    global allcfs
    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]
    start()
