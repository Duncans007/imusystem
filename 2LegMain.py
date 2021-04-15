#!/usr/bin/env python3

#Importing Custom Functions
from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)
from kneelingAlgorithm import * #kneelingDetection.kneelingDetection(objRT, objRS, objRH, objLT, objLS, objLH)
from CUNYreceiver import *
from NUCreceiver import *
from ARDUINOreceiver import *
from userinput import *
from variableDeclarations import *


#Importing python libraries
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from multiprocessing import Process,Queue,Pipe
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np
import sys


#Globalises all variables used inside the OSC reader b/c it is a separate process of sorts and allows variables to retain state between loops.
#These will all be passed, rather than global eventually.
global timeCurrent, varType, timeStart
global dataDict, flagDict, toggleFlagDict
global packetReady, rPacketReady, passToAlgorithm, packetWasReady
global fileDump
global objRHeel, objRShank, objRThigh
global objLHeel, objLShank, objLThigh
global gaitDetectRight, gaitDetectLeft
global objects
global hip_heel_length
global intelNUCserial, streamGait
global teensySend, teensyPort
global cuny_data
global alpha2, SecondsToChange, loadcell_data, loadCell


#Takes input arguments and switches appropriate booleans for running.
#Add toLower to standardsize
#Move to userinput.py file
if str((sys.argv)[1]) == "true":
    nucSend = True
if str((sys.argv)[1]) == "false":
    nucSend = False
if str((sys.argv)[2]) == "true":
    teensySend = True
if str((sys.argv)[2]) == "false":
    teensySend = False
if str((sys.argv)[3]) == "true":
    viconData = True
    teensySend = False
    teensyPort = serial.Serial(teensyPort, teensyBaud, timeout=3.0)
    parent_conn_teensy,child_conn_teensy = Pipe()
    p_teensy = Process(target=async_teensy, args=(child_conn_teensy, teensyPort))
    p_teensy.start()
if str((sys.argv)[3]) == "false":
    viconData = False



#Turns data collection for particular sensors on/off if necessary.
#Move to userinput.py file and standardize so system can be run with any # of sensors
toggleFlagDict = {
    "rThigh": True,
    "rShank": True,
    "rHeel": True,
    "lThigh": True,
    "lShank": True,
    "lHeel": True,
    "lowBack": True,
    "topBack": sensor8
}


#Variable initializations that can't be offloaded to another file. (time)
timeCurrent = time.time()
timeStart = time.time()


#Function to handle OSC input
def data_handler(address, *args):
    global timeCurrent, varType, timeStart
    global dataDict, flagDict, toggleFlagDict
    global packetReady, rPacketReady, passToAlgorithm, packetWasReady
    global fileDump
    global objRHeel, objRShank, objRThigh
    global objLHeel, objLShank, objLThigh
    global gaitDetectRight, gaitDetectLeft
    global objects
    global hip_heel_length
    global intelNUCserial, streamGait
    global teensySend, teensyPort
    global parent_conn, viconData
    global cuny_data
    global SecondsToChange, alpha2, loadcell_data, loadCell

    
#Pull data from [CUNY teensy / Rutgers VICON / Chadi Load Cell] if enabled
    if teensySend:
        if parent_conn_teensy.poll(0):
            cuny_data = parent_conn_teensy.recv()
        
    if viconData:
        if parent_conn_nuc.poll(0):
            nuc_data = parent_conn_nuc.recv()
            
    if loadCell:
        if parent_conn_arduino.poll(0):
            loadcell_data = parent_conn_arduino.recv()
            
    
    

	
#Collects variable type and sensor address as numbers
    out = []
    varType = address[10]
    addr = ''
    addr += str(address[len(address) - 3])
    addr += str(address[len(address) - 1])
    
    
#Takes in individual datapoints and assembles into easily indexable dictionary packages.
    if addr in addressDict:
        limb = addressDict[addr]

        if varType == "r":
            dataDict[limb] = package_handler_raw(args)
            
            if (limb == "topBack"):
                if (toggleFlagDict['topBack'] == True):
                    flagDict[limb] = True
            else:
                flagDict[limb] = True

            
#Tests if all sensors have been received before assembling packet and sending to algorithm
        if flagDict == toggleFlagDict:                    
            for x in flagDict:
                flagDict[x] = False
            
            rPacketReady = True
    
    
#Waits until full packet is assembled with all sensors, so that entire IMU state can be analyzed by algorithm at once.
    if rPacketReady:
        rPacketReady = False
        
        passToAlgorithm["rt_raw"] = dataDict["rThigh"]
        passToAlgorithm["rs_raw"] = dataDict["rShank"]
        passToAlgorithm["rh_raw"] = dataDict["rHeel"]
        passToAlgorithm["lt_raw"] = dataDict["lThigh"]
        passToAlgorithm["ls_raw"] = dataDict["lShank"]
        passToAlgorithm["lh_raw"] = dataDict["lHeel"]
        passToAlgorithm["b_raw"]  = dataDict["lowBack"]
        passToAlgorithm["tb_raw"] = dataDict["topBack"]
        
        packetReady = True
        
        
        
        
#----------------------------------------------------------------------------------------------------------------#    
#Code is broken into reader above and algorithms below for increased customization and ease of changing algorithm. Everything below this line is almost entirely customizable.
#If sensor orientations change, they can be changed in the code below.
        
#When complete system state is ready, run calculations
    if packetReady:
        packetWasReady = True
        packetReady = False
        timeLastRun = timeCurrent
        timeCurrent = time.time()
        timeToRun = timeCurrent - timeLastRun
        
#Update data in individual sensor objects 
        objRThigh.newValues(passToAlgorithm['rt_raw'])
        objRShank.newValues(passToAlgorithm['rs_raw'])
        objRHeel.newValues(passToAlgorithm['rh_raw'])
        objLThigh.newValues(passToAlgorithm['lt_raw'])
        objLShank.newValues(passToAlgorithm['ls_raw'])
        objLHeel.newValues(passToAlgorithm['lh_raw'])
        objLowBack.newValues(passToAlgorithm['b_raw'])
        objTopBack.newValues(passToAlgorithm['tb_raw'])
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#RUN CALCULATIONS -------------------------------------------------------------------------------------------------------------

#Calibrations - subject must stand still in a natural standing position. Whatever position sensors are in is zero position.
#Gyroscope calibration (function included in sensor object)
        if (time.time() - timeStart) < sensorCalibTime:
            objRThigh.getCalib()
            objRShank.getCalib()
            objRHeel.getCalib()

            objLThigh.getCalib()
            objLShank.getCalib()
            objLHeel.getCalib()
            
            objLowBack.getCalib()
            if toggleFlagDict['topBack'] == True:
                objTopBack.getCalib()

#Run angle zeroing (function included in sensor object)
        elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
            objRThigh.angleCalib()
            objRShank.angleCalib()
            objRHeel.angleCalib()
            
            objLThigh.angleCalib()
            objLShank.angleCalib()
            objLHeel.angleCalib()

            objLowBack.angleCalib()
            if toggleFlagDict['topBack'] == True:
                objTopBack.angleCalib()

        else:
    
#Run angle calculations for each individual sensor (function included in sensor object)
            objRThigh.angleCalc()
            objRShank.angleCalc()
            objRHeel.angleCalc()

            objLThigh.angleCalc()
            objLShank.angleCalc()
            objLHeel.angleCalc()
            
            objLowBack.angleCalc()
            if toggleFlagDict['topBack'] == True:
                objTopBack.angleCalc()

            
#-----------------------------------------------------------
#NO CALCULATIONS BEFORE ANGLECALC() OTHERWISE THEY WILL RUN USING RAW DATA INSTEAD OF PROPER UNITS

#Algorithms and secondary angle calculations
	
#Right and Left Gait Detection
        gaitDetectRight.testVal(objRThigh.gyZ, objRShank.gyZ, objRHeel.gyZ)
        gaitDetectLeft.testVal(objLThigh.gyZ, objLShank.gyZ, objLHeel.gyZ)

#Slip Algorithm - Calculates Slip Indicator from Trkov IFAC 2017 paper
        slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos(objRHeel.zAngleZeroed * .01745)) - (objRHeel.acY * np.sin(objRHeel.zAngleZeroed * .01745))), hip_heel_length)
        slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, ((objLHeel.acX * np.cos(objLHeel.zAngleZeroed * .01745)) - (objLHeel.acY * np.sin(objLHeel.zAngleZeroed * .01745))), hip_heel_length)

#Run Kneeling Detection Algorithm
        #legForward, kneeAngleR, kneeAngleL = kneelingDetect.kneelingDetection(objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel)
        #if (time.time() - timeStart > SecondsToChange):
        #    kneelingDetect.alpha = alpha2
        if viconData:
            kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorqueFromVicon(objRThigh, objRShank, objLThigh, objLShank, nuc_data["R"], nuc_data["L"], nuc_data["B"])
        else:
            kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorque(objRThigh, objRShank, objLThigh, objLShank, objLowBack)
        if streamGait:
           send_to_brace(gaitDetectLeft.gaitOutput, gaitSerial)
     
#DATA OUTPUT -------------------------------------------------------------------------------------------------------------

#Create beginning of output string - time, time between measurements, right gait stage, left gait stage, left slip detector, right slip detector
        outputString = f"{time.time() - timeStart}\t{timeToRun}\t{gaitDetectRight.gaitStage}\t{gaitDetectLeft.gaitStage}\t{slipRight}\t{slipLeft}\t{legForward}\t\t"


#Cycle through all sensor objects to append formatted version of every sensor's raw data to output string
        for x in objects:
            outputString += f"{x.gyX}\t"
            outputString += f"{x.gyY}\t"
            outputString += f"{x.gyZ}\t"

            outputString += f"{x.acX}\t"
            outputString += f"{x.acY}\t"
            outputString += f"{x.acZ}\t"

            outputString += f"{x.zAngleZeroed}\t"

            #outputString += f"{x.mgX}\t"
            #outputString += f"{x.mgY}\t"
            #outputString += f"{x.mgZ}\t\t"
			
            outputString += f"\t"

        #for x in objects:
        #    outputString += f"{x.gravAngleSmoothed}\t"
        #    outputString += f"{x.angleFromGravity}\t\t"
        if viconData:
            torqueROG = 0
            torqueLOG = 0

        outputString += f"{kneeAngleR}\t{kneeAngleL}\t{kneelingTorqueEstimationR}\t{kneelingTorqueEstimationL}"
        
        if loadCell:
            outputString += f"\t{loadcell_data[0]}\t"
        
        if teensySend:
            for x in cuny_data.values():
                outputString += f"{x}\t"

        outputString += f"\n"
		
        if nucSend == False:
            if loadCell:
                print(f"Read Rate: {1/timeToRun}\t{loadcell_data}")
            else:
                print(f"Read Rate: {1/timeToRun}\t{kneeAngleR}\t{kneeAngleL}\t{gaitDetectLeft.gaitStage}")
        
            
        fileDump.write(f"{outputString}")
		

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#SERIAL SEND--------------------------------------------
#IMPORTANT: msgArray NEW FORMAT IN ACCORDANCE WITH ALBORZ COMMUNICATION PROTOCOL
#[ 111, time,
#  LHAX, LHAY, LHAZ, LHGX, LHGY, LHGZ, LHAngle,      RHAX, RHAY, RHAZ, RHGX, RHGY, RHGZ, RHAngle,
#  LSAX, LSAY, LSAZ, LSGX, LSGY, LSGZ, LSAngle,      RSAX, RSAY, RSAZ, RSGX, RSGY, RSGZ, RSAngle,
#  LTAX, LTAY, LTAZ, LTGX, LTGY, LTGZ, LTAngle,      RTAX, RTAY, RTAZ, RTGX, RTGY, RTGZ, RTAngle,
#  LBAX, LBAY, LBAZ, LBGX, LBGY, LBGZ, LBAngle,
#  gaitL, gaitR, slipL, slipR, TorqueL, TorqueR ]

#To extract values:
#Torque = x * 0.002
#Angle = x * .0125
#Accelerometer = x * 0.002394
#Gyroscope = x * 0.07
#Magnetometer = x * 0.00014

        if nucSend:
            #serialArr = [time.time() - timeStart]
            serialArr = [1.0/timeToRun]
            for x in [objLHeel, objRHeel, objLShank, objRShank, objLThigh, objRThigh, objLowBack]:
                serialArr += [int(x.acX_norm/2), int(x.acY_norm/2), int(x.acZ_norm/2), int(x.gyX_norm/2), int(x.gyY_norm/2), int(x.gyZ_norm/2), int(x.zAngleZeroed * 80)]
            serialArr += [int(gaitDetectRight.gaitStage), int(gaitDetectLeft.gaitStage), int(slipRight/(10**32)), int(slipLeft/(10**32)), int(kneelingTorqueEstimationL * 500), int(kneelingTorqueEstimationR * 500)]
            #serialArr += [gaitDetectRight.gaitStage, gaitDetectLeft.gaitStage, int(slipRight/(10**32)), int(slipLeft/(10**32)), int(kneelingTorqueEstimationR * 500)]
            
            
            if teensySend:
                for i in cuny_data.items():
                    serialArr.append(int(i[1]))
                    
                    
            print(f"Read Rate: {1/timeToRun}") #print(serialArr)
            #print(serialArr)

            #print(f"Read Rate: {1/timeToRun}") #print(serialArr)
            #print([objLowBack.zAngle, objLThigh.zAngle, objRThigh.zAngle])
            print("%9.5f %9.5f %9.5f" %(1.0/timeToRun, kneelingTorqueEstimationL, kneelingTorqueEstimationR))
            send_over_serial(serialArr, intelNUCserial)
#-----------------------------------------------------
        if teensySend:
            if addGravityToTorque:
                send_to_teensy(kneelingTorqueEstimationL + cuny_data["ActTqL"], kneelingTorqueEstimationR + cuny_data["ActTqR"], teensyPort)
            else:
                send_to_teensy(kneelingTorqueEstimationL, kneelingTorqueEstimationR, teensyPort)
                
        elif viconData:
            if addGravityToTorque:
                send_to_teensy(kneelingTorqueEstimationL + cuny_data["ActTqL"], kneelingTorqueEstimationR + cuny_data["ActTqR"], teensyPort)
            else:
                send_to_teensy(kneelingTorqueEstimationL, kneelingTorqueEstimationR, teensyPort)
            
#-----------------------------------------------------








#Handles any OSC messages that aren't picked up by dataHandler (doesn't do anything with them.)
def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)


    
    
    
    
    
#Sets up OSC server
def main_func(ip, port):
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/r*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever


    
    
    
    
    
    
    

if __name__ == "__main__":
    #Variable initializations
    
    #Creation of objects for communication with Rutgers NUC device
    if nucSend:
        intelNUCserial = serial.Serial(intelNUCport, intelNUCbaud, timeout=3.0)
        if viconData:
            parent_conn_nuc,child_conn_nuc = Pipe()
            p_nuc = Process(target=async_nuc, args=(child_conn_nuc, intelNUCserial))
            p_nuc.start()
            
    if streamGait:
        gaitSerial = serial.Serial(arduinoPort, arduinoBaud, timeout=3.0)
	
    
    #Creation of objects for communication with CUNY Teensy device
    if teensySend:
        teensyPort = serial.Serial(teensyPort, teensyBaud, timeout=3.0)
        parent_conn_teensy,child_conn_teensy = Pipe()
        p_teensy = Process(target=async_teensy, args=(child_conn_teensy, teensyPort))
        p_teensy.start()
        
        
    #Creation of objects for communication with Chadi load cell via USB Arduino
    if loadCell:
        arduinoPort = serial.Serial(arduinoPort, arduinoBaud, timeout=3.0)
        parent_conn_arduino,child_conn_arduino = Pipe()
        p_arduino = Process(target=async_arduino, args=(child_conn_arduino, arduinoPort))
        p_arduino.start()
        loadcell_data = 0
    
    
    
    
    
    
    
    
#Create sensor objects to store and manipulate data for each sensor
    objRThigh = sensorObject("RT")
    objRShank = sensorObject("RS")
    objRHeel = sensorObject("RH")

    objLThigh = sensorObject("LT")
    objLShank = sensorObject("LS")
    objLHeel = sensorObject("LH")

    objLowBack = sensorObject("LB")
    objTopBack = sensorObject("LB")

    
#create gait detect objects for each leg
    gaitDetectRight = gaitDetect()
    gaitDetectLeft = gaitDetect()
    kneelingDetect = kneelingDetection(NMKG, mass, height, alpha, torqueCutoff, ramping_delay_time, ramping_hold_time, ramping_slope, controller_type, front_leg_proportion, rear_leg_proportion, back_proportion, back_offset)

    
#Create lists that can be cycled through to iterate over every object for exporting (and creating the dump file data header).
    if toggleFlagDict['topBack'] == True:
        objects = [objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel, objLowBack, objTopBack]
        stringObjects = ["RThigh", "RShank", "RHeel", "LThigh", "LShank", "LHeel", "LowBack", "TopBack"]
    else:
        objects = [objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel, objLowBack]
        stringObjects = ["RThigh", "RShank", "RHeel", "LThigh", "LShank", "LHeel", "LowBack"]
        
    stringAxes = ["x","y","z"]
    #stringSensors = ["gy","ac","mg"]
    stringSensors = ["gy","ac"]
    #Create formatted file header
    fileDump = open("algDump.txt", "w+")
    header = "time\ttimeToRun\tgaitStageR\tgaitStageL\tslipR\tslipL\tKneelingIndicator\t\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{y}/{z}/{x}\t"
        header += f"Angle/{x}\t"
        header += f"\t"

    header += f"KneeAngleR\tKneeAngleL\tKneeTorqueR\tKneeTorqueL"

    header += f"\n"
    fileDump.write(header)


#Start program
    main_func(ip, port)
    client.close()
