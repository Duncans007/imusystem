#!/usr/bin/env python3

import time
from sensorClass import *
from kneelingAlgorithm import *
from userinput import *
import sys

#1: import data from file
#2: get file number of rows
#3: start loop
#4: put data from line into objects
#5: angle calcs
#6: knee calcs
#Time step .02

NMKG = 0.15

objRThigh = sensorObject("RT")
objRShank = sensorObject("RS")
objLThigh = sensorObject("LT")
objLShank = sensorObject("LS")
objLoBack = sensorObject("LB")

kneelingDetect = kneelingDetection(NMKG, mass, height, alpha, torqueCutoff)

rFile = open('standaloneKneelingData.txt')
data = rFile.readlines()
count = len(data)
rFile.close()


wFile = open('algDump.txt', 'w+')



for i in range(count):
    outputStr = ''
    y = data[i].split()
    objLThigh.gyX = float(y[0])/2
    objLThigh.gyY = float(y[1])/2
    objLThigh.gyZ = float(y[2])/2
    objLThigh.acX = float(y[3])/2
    objLThigh.acY = float(y[4])/2
    objLThigh.acZ = float(y[5])/2

    objRThigh.gyX = float(y[6])/2
    objRThigh.gyY = float(y[7])/2
    objRThigh.gyZ = float(y[8])/2
    objRThigh.acX = float(y[9])/2
    objRThigh.acY = float(y[10])/2
    objRThigh.acZ = float(y[11])/2

    objLShank.gyX = float(y[12])/2
    objLShank.gyY = float(y[13])/2
    objLShank.gyZ = float(y[14])/2
    objLShank.acX = float(y[15])/2
    objLShank.acY = float(y[16])/2
    objLShank.acZ = float(y[17])/2

    objRShank.gyX = float(y[18])/2
    objRShank.gyY = float(y[19])/2
    objRShank.gyZ = float(y[20])/2
    objRShank.acX = float(y[21])/2
    objRShank.acY = float(y[22])/2
    objRShank.acZ = float(y[23])/2
    
    objLoBack.gyX = float(y[24])/2
    objLoBack.gyY = float(y[25])/2
    objLoBack.gyZ = float(y[26])/2
    objLoBack.acX = float(y[27])/2
    objLoBack.acY = float(y[28])/2
    objLoBack.acZ = float(y[29])/2
        
    if i < 50:
        objRThigh.getCalib()
        objRShank.getCalib()

        objLThigh.getCalib()
        objLShank.getCalib()
        
        objLoBack.getCalib()

    else:

        objRThigh.angleCalc()
        objRShank.angleCalc()

        objLThigh.angleCalc()
        objLShank.angleCalc()
        
        objLoBack.angleCalc()



    kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorque(objRThigh, objRShank, objLThigh, objLShank)
    outputStr = f"{objLThigh.zAngle}\t{objRThigh.zAngle}\t{objLShank.zAngle}\t{objRShank.zAngle}\t{kneelingTorqueEstimationL}\t{kneelingTorqueEstimationR}\t{kneeAngleL}\t{kneeAngleR}\t{legForward}\n"
    wFile.write(outputStr)
    print(outputStr)
    time.sleep(.02)
wFile.close()
