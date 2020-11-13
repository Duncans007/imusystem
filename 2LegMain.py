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
from userinput import *

#Importing python libraries
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from multiprocessing import Process,Queue,Pipe
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np
import sys

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
global intelNUCserial
global teensySend, teensyPort
global cuny_data, nuc_data

#ALL USER INPUT VARIABLES HAVE BEEN MOVED TO USERINPUT.PY
#USER INPUTS
ip = "localhost"
port = 6565

nucSend = True
viconData = True
teensySend = True

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
if str((sys.argv)[3]) == "false":
    viconData = False



#Turns data collection for particular sensors on/off if necessary.
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

#Variable Initializations

packetReady = False
rPacketReady = False
packetWasReady = True
timeCurrent = time.time()
timeStart = time.time()


dataDict = {
    "rThigh":  [],
    "rShank":  [],
    "rHeel":  [],
    "lThigh": [],
    "lShank": [],
    "lHeel": [],
    "lowBack":  [],
    "topBack": [0,0,0,0,0,0,0,0,0]
}


cuny_data = {
    "ActTqL" : 0,
    "ActTqR" : 0,
    "KneeL"  : 0,
    "KneeR"  : 0,
    "LTangX" : 0,
    "LTangY" : 0,
    "LTangZ" : 0,
    "LTgyX"  : 0,
    "LTgyY"  : 0,
    "LTgyZ"  : 0,
    "LTacX"  : 0,
    "LTacY"  : 0,
    "LTacZ"  : 0,
    "RTangX" : 0,
    "RTangY" : 0,
    "RTangZ" : 0,
    "RTgyX"  : 0,
    "RTgyY"  : 0,
    "RTgyZ"  : 0,
    "RTacX"  : 0,
    "RTacY"  : 0,
    "RTacZ"  : 0,
    "LSangX" : 0,
    "LSangY" : 0,
    "LSangZ" : 0,
    "LSgyX"  : 0,
    "LSgyY"  : 0,
    "LSgyZ"  : 0,
    "LSacX"  : 0,
    "LSacY"  : 0,
    "LSacZ"  : 0,
    "RSangX" : 0,
    "RSangY" : 0,
    "RSangZ" : 0,
    "RSgyX"  : 0,
    "RSgyY"  : 0,
    "RSgyZ"  : 0,
    "RSacX"  : 0,
    "RSacY"  : 0,
    "RSacZ"  : 0,
    "BangX"  : 0,
    "BangY"  : 0,
    "BangZ"  : 0,
    "BgyX"   : 0,
    "BgyY"   : 0,
    "BgyZ"   : 0,
    "BacX"   : 0,
    "BacY"   : 0,
    "BacZ"   : 0
}


nuc_data = {
    "L"  : 0,
    "R"  : 0,
    "B"   : 0
}


flagDict = {
    "rThigh": False,
    "rShank": False,
    "rHeel": False,
    "lThigh": False,
    "lShank": False,
    "lHeel": False,
    "lowBack": False,
    "topBack": False
}


addressDict = {
    "10": "rThigh",
    "11": "rShank",
    "12": "rHeel",
    "30": "lThigh",
    "31": "lShank",
    "32": "lHeel",
    "20": "lowBack",
    "52": "topBack"
}


orderDict = {
    0: "rThigh",
    1: "rShank",
    2: "rHeel",
    3: "lThigh",
    4: "lShank",
    5: "lHeel",
    6: "lowBack",
    7: "topBack"
}


passToAlgorithm = {
    "rt_raw": [],
    "rs_raw": [],
    "rh_raw": [],
    "lt_raw": [],
    "ls_raw": [],
    "lh_raw": [],
    "b_raw": [],
    "tb_raw": []
}


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
    global intelNUCserial, nucSend
    global teensySend, teensyPort
    global parent_conn, viconData
    global cuny_data, nuc_data   

	
#Collects variable type and sensor address as numbers
    out = []
    varType = address[10]
    addr = ''
    addr += str(address[len(address) - 3])
    addr += str(address[len(address) - 1])
    
    
#Takes in individual data and assembles into easily indexable dictionary packages.
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
    
    
#Assembles full packet so that entire IMU state can be analyzed by algorithm at once.
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
        
#Update data        
        objRThigh.newValues(passToAlgorithm['rt_raw'])
        objRShank.newValues(passToAlgorithm['rs_raw'])
        objRHeel.newValues(passToAlgorithm['rh_raw'])
        objLThigh.newValues(passToAlgorithm['lt_raw'])
        objLShank.newValues(passToAlgorithm['ls_raw'])
        objLHeel.newValues(passToAlgorithm['lh_raw'])
        objLowBack.newValues(passToAlgorithm['b_raw'])
        objTopBack.newValues(passToAlgorithm['tb_raw'])
        
        
        if teensySend:
            if parent_conn_teensy.poll(0):
                cuny_data = parent_conn_teensy.recv()

        if viconData:
            if parent_conn_nuc.poll(0):
                nuc_data = parent_conn_nuc.recv()
                print(nuc_data)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#RUN CALCULATIONS -------------------------------------------------------------------------------------------------------------


        if (time.time() - timeStart) < 1:
            objRThigh.getCalib()
            objRShank.getCalib()
            objRHeel.getCalib()

            objLThigh.getCalib()
            objLShank.getCalib()
            objLHeel.getCalib()
            
            objLowBack.getCalib()
            if toggleFlagDict['topBack'] == True:
                objTopBack.getCalib()

        else:
    #Right Leg Angle Approximations
            objRThigh.angleCalc()
            objRShank.angleCalc()
            objRHeel.angleCalc()

    #Left Leg Angle Approximations
            objLThigh.angleCalc()
            objLShank.angleCalc()
            objLHeel.angleCalc()
            
            objLowBack.angleCalc()
            if toggleFlagDict['topBack'] == True:
                objTopBack.angleCalc()

            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
#-----------------------------------------------------------
#NO CALCULATIONS BEFORE ANGLECALC() OTHERWISE THEY WILL RUN USING RAW DATA INSTEAD OF PROPER UNITS
	
#Right and Left Gait Detection
        gaitDetectRight.testVal(objRThigh.gyZ, objRShank.gyZ, objRHeel.gyZ)
        gaitDetectLeft.testVal(objLThigh.gyZ, objLShank.gyZ, objLHeel.gyZ)

#Calculates Slip Indicator from Trkov IFAC 2017 paper
        slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos(objRHeel.zAngle * .01745)) - (objRHeel.acY * np.sin(objRHeel.zAngle * .01745))), hip_heel_length)
        slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, ((objLHeel.acX * np.cos(objLHeel.zAngle * .01745)) - (objLHeel.acY * np.sin(objLHeel.zAngle * .01745))), hip_heel_length)

#Run Kneeling Detection Algorithm
        #legForward, kneeAngleR, kneeAngleL = kneelingDetect.kneelingDetection(objRThigh, objRShank, objRHeel, objLThigh, objLShank, objLHeel)
        if viconData:
            kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorqueFromVicon(objRThigh, objRShank, objLThigh, objLShank, nuc_data["R"], nuc_data["L"], nuc_data["B"])
        else:
            kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorque(objRThigh, objRShank, objLThigh, objLShank, objLowBack)



        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#PRINT TO OUTPUT STRING -------------------------------------------------------------------------------------------------------------

#Append beginning of output string - time, time between measurements, right gait stage, left gait stage, left slip detector, right slip detector
        outputString = f"{time.time() - timeStart}\t{timeToRun}\t{gaitDetectRight.gaitStage}\t{gaitDetectLeft.gaitStage}\t{slipRight}\t{slipLeft}\t{legForward}\t\t"

#Cycle through all sensor objects to append formatted version of every sensor's raw data to output string
        for x in objects:
            outputString += f"{x.gyX}\t"
            outputString += f"{x.gyY}\t"
            outputString += f"{x.gyZ}\t"

            outputString += f"{x.acX}\t"
            outputString += f"{x.acY}\t"
            outputString += f"{x.acZ}\t"

            outputString += f"{x.zAngle}\t"

            #outputString += f"{x.mgX}\t"
            #outputString += f"{x.mgY}\t"
            #outputString += f"{x.mgZ}\t\t"
			
            outputString += f"\t"

        #for x in objects:
        #    outputString += f"{x.gravAngleSmoothed}\t"
        #    outputString += f"{x.angleFromGravity}\t\t"

        outputString += f"{kneeAngleR}\t{kneeAngleL}\t{kneelingTorqueEstimationR}\t{kneelingTorqueEstimationL}"
        
        if teensySend:
            for x in cuny_data.values():
                outputString += f"{x}\t"

        outputString += f"\n"
		
        if nucSend == False:
            print(f"Read Rate: {1/timeToRun}\t{kneeAngleR}\t{kneeAngleL}")
            
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
            serialArr = [time.time() - timeStart]
            for x in [objLHeel, objRHeel, objLShank, objRShank, objLThigh, objRThigh, objLowBack]:
                serialArr += [int(x.acX_norm/2), int(x.acY_norm/2), int(x.acZ_norm/2), int(x.gyX_norm/2), int(x.gyY_norm/2), int(x.gyZ_norm/2), int(x.zAngle * 80)]
            serialArr += [int(gaitDetectRight.gaitStage), int(gaitDetectLeft.gaitStage), int(slipRight/(10**32)), int(slipLeft/(10**32)), int(kneelingTorqueEstimationL * 500), int(kneelingTorqueEstimationR * 500)]
            #serialArr += [gaitDetectRight.gaitStage, gaitDetectLeft.gaitStage, int(slipRight/(10**32)), int(slipLeft/(10**32)), int(kneelingTorqueEstimationR * 500)]
            
            
            if teensySend:
                for i in cuny_data.items():
                    serialArr.append(int(i[1]))
                    
                    
            print(f"Read Rate: {1/timeToRun}") #print(serialArr)
            send_over_serial(serialArr, intelNUCserial)
#-----------------------------------------------------
        #if teensySend:
        #    if addGravityToTorque:
        #        send_to_teensy(kneelingTorqueEstimationL + cuny_data["ActTqL"], kneelingTorqueEstimationR + cuny_data["ActTqR"], teensyPort)
        #    else:
        #        send_to_teensy(kneelingTorqueEstimationL, kneelingTorqueEstimationR, teensyPort)
                
                
    time.sleep(0.0001)




#Handles any OSC messages that aren't picked up by dataHandler
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
    #serial object for NUC. Comment out if not used.
    if nucSend:
        intelNUCserial = serial.Serial(intelNUCport, intelNUCbaud, timeout=3.0)
        if viconData:
            parent_conn_nuc,child_conn_nuc = Pipe()
            p_nuc = Process(target=async_nuc, args=(child_conn_nuc, intelNUCserial))
            p_nuc.start()
	
    

    if teensySend:
        teensyPort = serial.Serial(teensyPort, teensyBaud, timeout=3.0)
        parent_conn_teensy,child_conn_teensy = Pipe()
        p_teensy = Process(target=async_teensy, args=(child_conn_teensy, teensyPort))
        p_teensy.start()
    
    
    
    
    
    
    
    

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
    kneelingDetect = kneelingDetection(NMKG, mass, height, alpha, torqueCutoff, ramping_delay_time, ramping_hold_time, ramping_slope, controller_type)

    #create lists that can be cycles through to iterate over every object, as well as create the file data header.
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

    main_func(ip, port)
    client.close()
