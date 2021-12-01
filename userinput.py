#!/usr/bin/env python3	

#Frequency that the system outputs at (Hz)(recommended max 100)
processing_frequency = 50

#Activate/deactivate individual sensor recording
#Note that disabling some (e.g. shank) will cause some detectors to throw errors
toggle_rThigh  = True
toggle_rShank  = True
toggle_rHeel   = True
toggle_lThigh  = True
toggle_lShank  = True
toggle_lHeel   = True
toggle_lowBack = True
toggle_topBack = False


#Hip to heel length for Trkov slip detection (m)
hip_heel_length = 1


#Torque Controller Options: 
# "pid" - DS developed, uses knee angle and thigh ang. vel.
# "yusu" - Based on paper by Yu&Su. Uses thigh kinematics plus lower back.
# "ramp" - Constant ramping torque.
# "trkov" - Developed by MT, adapted by DS, uses full lower body kinematics plus lower back.
controller_type = "trkov"	

#General Torque Controller Parameters	
mass = 68 #Subject mass (kg)
height = 1.80 #Subject height (m)
torqueCutoff = 30 #Maximum allowable torque (Nm	) - pass to kneeling object
NMKG = 0.25 #Approximate torque per subject unit weight (Nm/kg)

#YU&SU Controller Proportionality Constants	
alpha = .1	

#Ramping Specific Constants	
ramping_delay_time = 5 #seconds	
ramping_hold_time = 5 #seconds	
ramping_slope = 20 #Nm/s	


#ADDITIONAL DEVICES
#Serial Send/Receive for NUC and Simulink	
nucSend = False
intelNUCport = "/dev/ttyUSB0"	
#intelNUCport = "/dev/ttyS0"	
intelNUCbaud = 115200	

#Load cell on foot
#Connects to arduino, which plugs into pi via USB port
#Configure arduinoPort/arduinoBaud, values are used for both loadCell and streamGait
loadCell = False #stream loadcell variables from arduino

#Arduino Gait Stream
#Arduino can relay information directly to the computer
#This is beneficial because it bypasses the need for a serial interpreter or wifi
#Data can easily be captured using putty or other simple software
streamGait = False #stream gait variables to arduino

arduinoPort = "/dev/ttyACM0"
arduinoBaud = 256000
