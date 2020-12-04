#!/usr/bin/env python3	


#Options: "pid","yusu","ramp", "trkov"
controller_type = "trkov"	


#General Torque Controller Parameters	
mass = 68 #kg	
height = 1.80 #m	
torqueCutoff = 30 #Nm	
NMKG = 0.25 #Nm/kg	


#Low Back Angle Proportionality (experimental)
back_proportion = 1 #0 degrees or usual operation
#back_proportion = 0.3432 #10 degrees
#back_proportion = 0.4456 #20 degrees


#PID Specific Constants (unused)
front_leg_proportion = 1;
rear_leg_proportion = 1;


#Yu&Su Specific Proportionality Constant	
alpha = .1	
alpha2 = 0.15	
SecondsToChange = 1	


#Ramping Specific Constants	
ramping_delay_time = 5 #seconds	
ramping_hold_time = 5 #seconds	
ramping_slope = 20 #Nm/s	


#Serial Send/Receive to NUC and Simulink	
intelNUCport = "/dev/ttyUSB0"	
#intelNUCport = "/dev/ttyS0"	
intelNUCbaud = 115200	


#Serial Send/Receive to CUNY Teensy	
teensyPort = "/dev/ttyACM0"	
#teensyPort = "/dev/ttyS0"
teensyBaud = 115200	


#Configure calibration times
sensorCalibTime = 2
angleCalibTime = 2	


#Turn upper back sensor on or off
sensor8 = False


hip_heel_length = 1 #meters	
