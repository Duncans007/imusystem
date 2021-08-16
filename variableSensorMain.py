#!/usr/bin/env python3

#Importing python libraries
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from multiprocessing import Process,Queue,Pipe
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np
import sys
import time

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

#Globalises all variables used inside the OSC reader b/c it is a separate process and allows variables to retain state between loops.
#These will all be passed, rather than global... eventually.
global timeCurrent, varType, timeStart
global dataDict, toggleFlagDict
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
global alpha2, SecondsToChange, loadcell_data, loadCell, timeLastRun
global calcTime


#Turns data collection for particular sensors on/off if necessary.
toggleFlagDict = {
    "rThigh":  toggle_rThigh,
    "rShank":  toggle_rShank,
    "rHeel":   toggle_rHeel,
    "lThigh":  toggle_lThigh,
    "lShank":  toggle_lShank,
    "lHeel":   toggle_lHeel,
    "lowBack": toggle_lowBack,
    "topBack": toggle_topBack
}

calcTime = [0]


#Variable initializations that can't be offloaded to another file (time)
timeCurrent = time.time()
timeStart = timeCurrent
timeLastRun = timeCurrent


#Function to handle OSC input
def data_handler(address, *args):
    global timeCurrent, varType, timeStart
    global dataDict, toggleFlagDict
    global passToAlgorithm, packetWasReady
    global fileDump
    global objRHeel, objRShank, objRThigh
    global objLHeel, objLShank, objLThigh
    global gaitDetectRight, gaitDetectLeft
    global objects
    global hip_heel_length
    global intelNUCserial, streamGait
    global teensySend, teensyPort
    global parent_conn
    global cuny_data
    global SecondsToChange, alpha2, loadcell_data, loadCell, timeLastRun
    global calcTime

###########################################################################################
#Pull data from [CUNY teensy / Chadi Load Cell] if enabled----------------------------------------------------------------------
    if teensySend:
        if parent_conn_teensy.poll(0):
            cuny_data = parent_conn_teensy.recv()
            
    if loadCell:
        if parent_conn_arduino.poll(0):
            loadcell_data = parent_conn_arduino.recv()

###########################################################################################
#PULL DATA FROM NOTOCHORD-----------------------------------------------------------------------------------------------------------------------------
#Collects variable type and sensor address
    out = []
    varType = address[10]
    addr = ''
    addr += str(address[len(address) - 3])
    addr += str(address[len(address) - 1])
    
    
#Takes in individual datapoints and assembles into easily indexable dictionary packages.
    if addr in addressDict:
        limb = addressDict[addr]

        if (varType == "r") and (toggleFlagDict[limb] == True):

            if limb == 'rThigh':
                objRThigh.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objRThigh.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objRThigh.getCalib()
                else:
                    objRThigh.angleCalc()
            
            if limb == 'rShank':
                objRShank.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objRShank.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objRShank.getCalib()
                else:
                    objRShank.angleCalc()
            
            if limb == 'rHeel':
                objRHeel.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objRHeel.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objRHeel.getCalib()
                else:
                    objRHeel.angleCalc()
            
            if limb == 'lThigh':
                objLThigh.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objLThigh.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objLThigh.getCalib()
                else:
                    objLThigh.angleCalc()
            
            if limb == 'lShank':
                objLShank.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objLShank.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objLShank.getCalib()
                else:
                    objLShank.angleCalc()
            
            if limb == 'lHeel':
                objLHeel.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objLHeel.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objLHeel.getCalib()
                else:
                    objLHeel.angleCalc()
            
            if limb == 'lowBack':
                objLowBack.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objLowBack.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objLowBack.getCalib()
                else:
                    objLowBack.angleCalc()
            
            if limb == 'topBack':
                objTopBack.newValues(package_handler_raw(args))
                if (time.time() - timeStart) < sensorCalibTime:
                    objTopBack.getCalib()
                elif (time.time() - timeStart >= sensorCalibTime) and (time.time() - timeStart < sensorCalibTime + angleCalibTime):
                    objTopBack.getCalib()
                else:
                    objTopBack.angleCalc()


###########################################################################################
#Auto-sends packet every 1/50 seconds regardless of packet completion status-----------------------------------------------------------------
    if (time.time() - timeCurrent) >= (1/processing_frequency):
        
        
        
###########################################################################################
#Algorithms and secondary angle calculations-------------------------------------------------------------------------------------------------
#Code is broken into reader above and algorithms below for increased customization and ease of changing algorithm. Everything below this line is almost entirely customizable.

#Timers
        timeLastRun = timeCurrent
        timeCurrent = time.time()
        timeToRun = timeCurrent - timeLastRun
        tic = time.perf_counter()

#Right and Left Gait Detection
        gaitDetectRight.testVal(objRThigh.gyZ, objRShank.gyZ, objRHeel.gyZ)
        gaitDetectLeft.testVal(objLThigh.gyZ, objLShank.gyZ, objLHeel.gyZ)

#Slip Algorithm - Calculates Slip Indicator from Trkov IFAC 2017 paper
        slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos(objRHeel.zAngleZeroed * .01745)) - (objRHeel.acY * np.sin(objRHeel.zAngleZeroed * .01745))), hip_heel_length)
        slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, ((objLHeel.acX * np.cos(objLHeel.zAngleZeroed * .01745)) - (objLHeel.acY * np.sin(objLHeel.zAngleZeroed * .01745))), hip_heel_length)

#Run Kneeling Detection Algorithm and torque estimator
        #legForward, kneeAngleR, kneeAngleL = kneelingDetect.kneelingDetection(objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel)
        #if (time.time() - timeStart > SecondsToChange):
        #    kneelingDetect.alpha = alpha2
        kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorque(objRThigh, objRShank, objLThigh, objLShank, objLowBack)


###########################################################################################
#DATA OUTPUT -------------------------------------------------------------------------------------------------------------

        if streamGait:
           send_to_brace(gaitDetectLeft.gaitOutput, gaitSerial)

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
		
  
###########################################################################################        
#SERIAL SEND---------------------------------------------------------------------------------------------------
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

            print("%9.5f %9.5f %9.5f" %(1.0/timeToRun, kneelingTorqueEstimationL, kneelingTorqueEstimationR))
            send_over_serial(serialArr, intelNUCserial)
#-----------------------------------------------------
        if teensySend:
            if addGravityToTorque:
                send_to_teensy(kneelingTorqueEstimationL + cuny_data["ActTqL"], kneelingTorqueEstimationR + cuny_data["ActTqR"], teensyPort)
            else:
                send_to_teensy(kneelingTorqueEstimationL, kneelingTorqueEstimationR, teensyPort)
        
#-----------------------------------------------------







###########################################################################################
#Handles any OSC messages that aren't picked up by dataHandler (doesn't do anything with them.)
def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)


    
    
    
    
###########################################################################################
#Set up OSC server
def main_func(ip, port):
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/r*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever


    
    
    
    
    
    
    
###########################################################################################
#Setup
if __name__ == "__main__":
#Variable initializations
    
#Creation of objects for communication with Rutgers NUC device
    if nucSend:
        intelNUCserial = serial.Serial(intelNUCport, intelNUCbaud, timeout=3.0)
    
#Creation of objects to stream gait variables to knee device arduino
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
    
#TODO: redo kneeling detect object
    kneelingDetect = kneelingDetection(NMKG, mass, height, alpha, torqueCutoff, ramping_delay_time, ramping_hold_time, ramping_slope, controller_type, front_leg_proportion, rear_leg_proportion, back_proportion, back_offset)

    
#Create lists that can be cycled through to iterate over every object for exporting (and creating the dump file data header).
    objects = []
    stringObjects = []
    
    if toggleFlagDict['rThigh'] == True:
        objects.append(objRThigh)
        stringObjects.append("RThigh")    
    if toggleFlagDict['rShank'] == True:
        objects.append(objRShank)
        stringObjects.append("RShank")    
    if toggleFlagDict['rHeel'] == True:
        objects.append(objRHeel)
        stringObjects.append("RHeel")    
    if toggleFlagDict['lThigh'] == True:
        objects.append(objLThigh)
        stringObjects.append("LThigh")    
    if toggleFlagDict['lShank'] == True:
        objects.append(objLShank)
        stringObjects.append("LShank")    
    if toggleFlagDict['lHeel'] == True:
        objects.append(objLHeel)
        stringObjects.append("LHeel")    
    if toggleFlagDict['lowBack'] == True:
        objects.append(objLowBack)
        stringObjects.append("LowBack")    
    if toggleFlagDict['topBack'] == True:
        objects.append(objTopBack)
        stringObjects.append("TopBack")
        
    stringAxes = ["x","y","z"]
    #stringSensors = ["gy","ac","mg"]
    stringSensors = ["gy","ac"]
    

#Create formatted file header
    fileDump = open("algDump.txt", "w+")
    header = "time\timeToRun\tgaitStageR\tgaitStageL\tslipR\tslipL\tKneelingIndicator\t\t"
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
