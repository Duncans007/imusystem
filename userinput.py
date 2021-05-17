#!/usr/bin/env python3	

#Activate/deactivate individual sensor recording
#Note that disabling some (e.g. shank) will cause some detectors to not work
toggle_rThigh  = True
toggle_rShank  = True
toggle_rHeel   = True
toggle_lThigh  = True
toggle_lShank  = True
toggle_lHeel   = True
toggle_lowBack = True
toggle_topBack = True


#Configure Sensor Calibration Times
sensorCalibTime = 2 #Calib time to eliminate gyro noise.
angleCalibTime = 2 #Calib time (after finishing sensorCalibTime) to zero angle measurements.


#Torque Controller Options: 
# "pid" - DS developed, uses knee angle and thigh ang. vel.
# "yusu" - Based on paper by Yu&Su. Uses thigh kinematics plus lower back.
# "ramp" - Constant ramping torque.
# "trkov" - Developed by MT, adapted by DS, uses full lower body kinematics plus lower back.
controller_type = "trkov"	


#General Torque Controller Parameters	
mass = 68 #Subject mass (kg)
height = 1.80 #Subject height (m)
torqueCutoff = 30 #Maximum allowable torque (Nm	)
NMKG = 0.25 #Approximate torque per subject unit weight (Nm/kg)


#YU&SU Controller Proportionality Constants	
alpha = .1	
alpha2 = 0.15	
SecondsToChange = 1


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


#Serial Send/Receive for CUNY Teensy	
teensySend = False
teensyPort = "/dev/ttyACM0"	
#teensyPort = "/dev/ttyS0"
teensyBaud = 115200	


#Load cell on foot
#Load cell plugs into arduino, which plugs into pi via USB port
#same values used for streaming gait variables to arduino
loadCell = False #stream loadcell variables from arduino
streamGait = False #stream gait variables to arduino
arduinoPort = "/dev/ttyACM0"
arduinoBaud = 256000

















#Low Back Angle Proportionality (EXPERIMENTAL)
back_proportion = 1 #0 degrees or usual operation
#back_proportion = 0.3432 #10 degrees
#back_proportion = 0.4456 #20 degrees


#PID controller constants (UNUSED)
front_leg_proportion = 1;
rear_leg_proportion = 1;


hip_heel_length = 1 #meters	


#[LEGACY] Back offset - used for configuring torque to slopes
#Has been auto-implemented
back_offset = 0
