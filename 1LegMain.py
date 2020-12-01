#!/usr/bin/env python3

from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np

global timeCurrent, varType, timeStart
global dataDict, flagDict, toggleFlagDict
global packetReady, rPacketReady, passToAlgorithm
global fileDump
global objRHeel, objRShank, objRThigh
global objLHeel, objLShank, objLThigh
global gaitDetectRight, gaitDetectLeft
global objects

#-----------------------------------#
ip = "localhost"
port = 6565

mass = 60 #kg
height = 180 #cm
#-----------------------------------#


toggleFlagDict = {
    "rThigh": True,
    "rShank": True,
    "rHeel": True,
    "lowBack": True,
}


#Variable Initializations

packetReady = False
rPacketReady = False
timeCurrent = time.time()
timeStart = time.time()


dataDict = {
    "rThigh":  [],
    "rShank":  [],
    "rHeel":  [],
    "lowBack":  [],
}


flagDict = {
    "rThigh": False,
    "rShank": False,
    "rHeel": False,
    "lowBack": False,
}


addressDict = {
    "10": "rThigh",
    "11": "rShank",
    "12": "rHeel",
    "20": "lowBack",
}


orderDict = {
    0: "rThigh",
    1: "rShank",
    2: "rHeel",
    3: "lowBack",
}


passToAlgorithm = {
    "rt_raw": [],
    "rs_raw": [],
    "rh_raw": [],
    "b_raw": [],
}


def data_handler(address, *args):
    global timeCurrent, varType, timeStart
    global dataDict, flagDict, toggleFlagDict
    global packetReady, rPacketReady, passToAlgorithm
    global fileDump
    global objRHeel, objRShank, objRThigh
    global objLHeel, objLShank, objLThigh
    global gaitDetectRight, gaitDetectLeft
    global objects
    
    out = []
    
    varType = address[10]
    
    addr = ''
    addr += str(address[len(address) - 3])
    addr += str(address[len(address) - 1])
    
    #Takes in individual data and assembles into packages
    if addr in addressDict:
        limb = addressDict[addr]

        if varType == "r":
            dataDict[limb] = package_handler_raw(args)
            flagDict[limb] = True

        if flagDict == toggleFlagDict:
            for x in range(len(flagDict) - 1):
                out.append(dataDict[orderDict[x]])
                    
            for x in flagDict:
                flagDict[x] = False
            
            rPacketReady = True
    
    
    #Assembles full packet for sending to Algorithm
    if rPacketReady:
        rPacketReady = False
        
        passToAlgorithm["rt_raw"] = dataDict["rThigh"]
        passToAlgorithm["rs_raw"] = dataDict["rShank"]
        passToAlgorithm["rh_raw"] = dataDict["rHeel"]
        passToAlgorithm["b_raw"]  = dataDict["lowBack"]
        
        packetReady = True
        
    #-------------------------------------------------------------#    
    #Code is broken into reader above and algorithm below for increased customization and ease of changing algorithm.
        
    
    if packetReady:
        packetReady = False
		
        timeLastRun = timeCurrent
        timeCurrent = time.time()
        timeToRun = timeCurrent - timeLastRun
        
        #Update data
        rt_raw = passToAlgorithm['rt_raw']
        rs_raw = passToAlgorithm['rs_raw']
        rh_raw = passToAlgorithm['rh_raw']
        b_raw  = passToAlgorithm['b_raw']

        #Values are moved around to equalize the axes of the IMUs.
        #Does not put IMUs on a global coordinate system.
        #Only sets local axes to be the same.

        #Right Thigh - no flipped values
        objRThigh.gyX = rt_raw[0]
        objRThigh.gyY = rt_raw[1]
        objRThigh.gyZ = rt_raw[2]
                  
        objRThigh.acX = rt_raw[3]
        objRThigh.acY = rt_raw[4]
        objRThigh.acZ = rt_raw[5]
                  
        objRThigh.mgX = rt_raw[6]
        objRThigh.mgY = rt_raw[7]
        objRThigh.mgZ = rt_raw[8]
        
        #Right Shank - no flipped values
        objRShank.gyX = rs_raw[0]
        objRShank.gyY = rs_raw[1]
        objRShank.gyZ = rs_raw[2]
                  
        objRShank.acX = rs_raw[3]
        objRShank.acY = rs_raw[4]
        objRShank.acZ = rs_raw[5]
                  
        objRShank.mgX = rs_raw[6]
        objRShank.mgY = rs_raw[7]
        objRShank.mgZ = rs_raw[8]
        
        #Right Heel - X and Y axes flipped, X negated
        objRHeel.gyX = -rh_raw[1]
        objRHeel.gyY = rh_raw[0]
        objRHeel.gyZ = rh_raw[2]
        
        objRHeel.acX = -rh_raw[4]
        objRHeel.acY = rh_raw[3]
        objRHeel.acZ = rh_raw[5]
        
        objRHeel.mgX = -rh_raw[7]
        objRHeel.mgY = rh_raw[6]
        objRHeel.mgZ = rh_raw[8]
        
        #Lower Back - X and Z values flipped, X value negated
        objLowBack.gyX = -b_raw[2]
        objLowBack.gyY =  b_raw[1]
        objLowBack.gyZ =  b_raw[0]
                       
        objLowBack.acX = -b_raw[5]
        objLowBack.acY =  b_raw[4]
        objLowBack.acZ =  b_raw[3]
                   
        objLowBack.mgX = -b_raw[8]
        objLowBack.mgY =  b_raw[7]
        objLowBack.mgZ =  b_raw[6]
        
        
        #Run actual calculations contained in the objects
        
        objRThigh.angleCalc(gaitDetectRight)
        objRShank.angleCalc(gaitDetectRight)
        objRHeel.angleCalc(gaitDetectRight)
        
        gaitDetectRight.testVal(objRShank.gyZ, objRHeel.gyZ)
        
        outputString = f"{time.time() - timeStart}\t{timeToRun}\t{gaitDetectRight.gaitStage}\t\t"
		
        for x in objects:
            outputString += f"{x.gyX}\t"
            outputString += f"{x.gyY}\t"
            outputString += f"{x.gyZ}\t\t"

            outputString += f"{x.acX}\t"
            outputString += f"{x.acY}\t"
            outputString += f"{x.acZ}\t\t"

            outputString += f"{x.mgX}\t"
            outputString += f"{x.mgY}\t"
            outputString += f"{x.mgZ}\t\t\t"

        for x in objects:
            outputString += f"{x.zAngle}\t"
	
        outputString += f"{gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos(objRHeel.zAngle * .01745)) - (objRHeel.acY * np.sin(objRHeel.zAngle * .01745))), 1)}\t"
		
        print(outputString)
        fileDump.write(f"{outputString}")

        fileDump.write("\n")



def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)
    #print(out)



def main_func(ip, port):   
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/r*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever



if __name__ == "__main__":    
    objRThigh = sensorObject()
    objRShank = sensorObject()
    objRHeel = sensorObject()
	
    objLowBack = sensorObject()
    
    gaitDetectRight = gaitDetect()
	
    objects = [objRThigh, objRShank, objRHeel, objLowBack]
    stringObjects = ["RThigh", "RShank", "RHeel", "LowBack"]
    stringAxes = ["x","y","z"]
    stringSensors = ["gy","ac","mg"]
	
	
    fileDump = open("algDump.txt", "w+")
    header = "time\ttimeToRun\tgaitStageR\t\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{y}/{z}/{x}\t"
            header += f"\t"
        header += f"\t"
	
    for x in stringObjects:
        header += f"zAngle{x}\t"
		
    header += f"slipRight\t"
	
    header += f"\n"
    fileDump.write(header)
    
    main_func(ip, port)
    client.close()
