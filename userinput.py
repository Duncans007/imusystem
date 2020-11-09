#!/usr/bin/env python3

#Torque Controller
mass = 80 #kg
height = 1.80 #m
torqueCutoff = 18 #Nm
alpha = .1 #proportionality
controller_type = "ramp"
#Option: "pid","yusu","ramp"

ramping_delay_time = 5 #seconds
ramping_hold_time = 5 #seconds
ramping_slope = 20 #Nm/s


#Serial Send/Receive to NUC and Simulink
#intelNUCport = "/dev/ttyUSB0"
intelNUCport = "/dev/ttyS0"
intelNUCbaud = 256000


#Serial Send/Receive to CUNY Teensy
#teensyPort = "/dev/ttyACM0"
teensyPort = "/dev/ttyS0"
teensyBaud = 256000


hip_heel_length = 1 #meters
NMKG = 0.15

sensor8 = False
