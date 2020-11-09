#!/usr/bin/env python3

#Option: "pid","yusu","ramp"
controller_type = "ramp"

#General Torque Controller Parameters
mass = 80 #kg
height = 1.80 #m
torqueCutoff = 18 #Nm
NMKG = 0.15 #Nm/kg

#Yu&Su Specific Proportionality Constant
alpha = .1

#Ramping Specific Constants
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

sensor8 = False
