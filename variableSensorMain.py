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



#Variable initializations that can't be offloaded to another file (time)
timeCurrent = time.time()
timeStart = timeCurrent
timeLastRun = timeCurrent


#Function to handle OSC input
def data_handler(address, *args):
    global timeCurrent, varType, timeStart
    global toggleFlagDict, fileDump
    global objRHeel, objRShank, objRThigh, objLHeel, objLShank, objLThigh
    global gaitDetectRight, gaitDetectLeft, objects
    global hip_heel_length
    global intelNUCserial, streamGait, teensySend, teensyPort, cuny_data
    global timeLastRun

    ###########################################################################################
    #Pull data from [Chadi Load Cell] if enabled----------------------------------------------------------------------
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
    tic = time.time()
    
    
    #Takes in individual datapoints and assembles into easily indexable dictionary packages.
    if addr in addressDict:
        limb = addressDict[addr]

        if (varType == "r") and (toggleFlagDict[limb] == True):

            if limb == 'rThigh':
                objRThigh.newValues(package_handler_raw(args))
            
            if limb == 'rShank':
                objRShank.newValues(package_handler_raw(args))
            
            if limb == 'rHeel':
                objRHeel.newValues(package_handler_raw(args))
            
            if limb == 'lThigh':
                objLThigh.newValues(package_handler_raw(args))
            
            if limb == 'lShank':
                objLShank.newValues(package_handler_raw(args))
            
            if limb == 'lHeel':
                objLHeel.newValues(package_handler_raw(args))
            
            if limb == 'lowBack':
                objLowBack.newValues(package_handler_raw(args))
            
            if limb == 'topBack':
                objTopBack.newValues(package_handler_raw(args))


    ###########################################################################################
    # Runs calculations every [processing_frequency] with whatever values are present -----------------------------------------------------------------
    if (time.time() - timeCurrent) >= (1/processing_frequency):
        
        
        
    ###########################################################################################
        #DETECTION ALGORITHMS AND OTHER SECONDARY CALCS -------------------------------------------------------------------------------------------------
        #Code is broken into reader above and algorithms below for increased customization and ease of changing algorithm. Everything below this line is almost entirely customizable.

        #Timers
        timeLastRun = timeCurrent
        timeCurrent = time.time()
        timeToRun = timeCurrent - timeLastRun

        #Right and Left Gait Detection
        gaitDetectRight.testVal(objRThigh.gyZ, objRShank.gyZ, objRHeel.gyZ)
        gaitDetectLeft.testVal(objLThigh.gyZ, objLShank.gyZ, objLHeel.gyZ)

        #Slip Algorithm - Calculates Slip Indicator from Trkov IFAC 2017 paper
        slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos(objRHeel.zAngleZeroed * .01745)) - (objRHeel.acY * np.sin(objRHeel.zAngleZeroed * .01745))), hip_heel_length)
        slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, ((objLHeel.acX * np.cos(objLHeel.zAngleZeroed * .01745)) - (objLHeel.acY * np.sin(objLHeel.zAngleZeroed * .01745))), hip_heel_length)

        #Run Kneeling Detection Algorithm and torque estimator
        kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorque(objRThigh, objRShank, objLThigh, objLShank, objLowBack)


    ###########################################################################################
        #DATA OUTPUT (FILE) -------------------------------------------------------------------------------------------------------------

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
            outputString += f"{x.xAngleZeroed}\t"

            #outputString += f"{x.mgX}\t"
            #outputString += f"{x.mgY}\t"
            #outputString += f"{x.mgZ}\t\t"
			
            outputString += f"\t"

        # Generate end of output string
        outputString += f"{kneeAngleR}\t{kneeAngleL}\t{kneelingTorqueEstimationR}\t{kneelingTorqueEstimationL}"
        if loadCell:
            outputString += f"\t{loadcell_data[0]}\t"
        outputString += f"\n"

        # Add output string to file
        fileDump.write(f"{outputString}")


    ###########################################################################################
        # DATA OUTPUT (CONSOLE) -------------------------------------------------------------------------------------------------------------


		# Print easy-reading values to terminal
        if not nucSend:
            if loadCell:
                print(f"Read Rate: {np.round(1/timeToRun, 2)}\t{loadcell_data}")
            else:
                ttr = time.time() - tic
                print(f"Read Rate: {np.round(1/timeToRun, 2)}\t{np.round(kneeAngleR, 2)}\t{np.round(kneeAngleL, 2)}\t{np.round(objRThigh.xAngleZeroed, 2)}\t{np.round(ttr, 5)}")


    ###########################################################################################
        # DATA OUTPUT (SERIAL SEND) ---------------------------------------------------------------------------------------------------

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

        # Compile and send data to MatLab Simulink program
        if nucSend:
            serialArr = [1.0/timeToRun]
            for x in [objLHeel, objRHeel, objLShank, objRShank, objLThigh, objRThigh, objLowBack]:
                serialArr += [int(x.acX_norm/2), int(x.acY_norm/2), int(x.acZ_norm/2), int(x.gyX_norm/2), int(x.gyY_norm/2), int(x.gyZ_norm/2), int(x.zAngleZeroed * 80)]
            serialArr += [int(gaitDetectRight.gaitStage), int(gaitDetectLeft.gaitStage), int(slipRight/(10**32)), int(slipLeft/(10**32)), int(kneelingTorqueEstimationL * 500), int(kneelingTorqueEstimationR * 500)]
                    
            print(f"Read Rate: {1/timeToRun}") #print(serialArr)

            #print("%9.5f %9.5f %9.5f" %(1.0/timeToRun, kneelingTorqueEstimationL, kneelingTorqueEstimationR))
            send_over_serial(serialArr, intelNUCserial)

###########################################################################################
    # DATA OUTPUT (Stream perturbed leg gait variables to brace arduino) ---------------------------------------------------------------------------------------------------
        
        if streamGait:
           send_to_brace(gaitDetectLeft.gaitOutput, gaitSerial)

## End of main reader function







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
    
    #Creation of objects for communication with Rutgers NUC device (MATLAB Simulink)
    if nucSend:
        intelNUCserial = serial.Serial(intelNUCport, intelNUCbaud, timeout=3.0)
    
    #Creation of objects to stream gait variables to knee device arduino
    if streamGait:
        gaitSerial = serial.Serial(arduinoPort, arduinoBaud, timeout=3.0)
        
    #Creation of objects for receiving data from Chadi load cell via USB Arduino
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
    kneelingDetect = kneelingDetection(NMKG, mass, height, alpha, torqueCutoff, ramping_delay_time, ramping_hold_time, ramping_slope, controller_type)

    
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
    #stringSensors = ["gy","ac","mg"] # Use in place of below line if adding magnetometer readings to file output
    stringSensors = ["gy","ac"]
    

    #Create formatted file header and write to file
    fileDump = open("algDump.txt", "w+")
    header = "time\ttimeToRun\tgaitStageR\tgaitStageL\tslipR\tslipL\tKneelingIndicator\t\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{y}/{z}/{x}\t"
        header += f"zAngle/{x}\txAngle/{x}\t"
        header += f"\t"

    header += f"KneeAngleR\tKneeAngleL\tKneeTorqueR\tKneeTorqueL"
    header += f"\n"
    fileDump.write(header)


    #Start execution
    main_func(ip, port)
    client.close()
