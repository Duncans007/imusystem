#!/usr/bin/env python3

#Torque Controller
mass = 80 #kg
height = 1.80 #m
torqueCutoff = 18 #Nm
alpha = .1 #proportionality


#Serial Send/Receive
#intelNUCport = "/dev/ttyUSB0"
intelNUCport = "/dev/ttyS0"
intelNUCbaud = 256000


teensyPort = "/dev/ttyS0"
teensyBaud = 256000

hip_heel_length = 1 #meters
NMKG = 0.15
