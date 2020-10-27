#!/usr/bin/env python3

import time
from sensorClass import *
from kneelingAlgorithm import *
import sys

#1: import data from file
#2: get file number of rows
#3: start loop
#4: put data from line into objects
#5: angle calcs
#6: knee calcs
#Time step .02

objRThigh = sensorObject("RT")
objRShank = sensorObject("RS")
objLThigh = sensorObject("LT")
objLShank = sensorObject("LS")

rFile = open('standaloneKneelingData.txt')
data = rFile.readlines()
count = len(data)
rFile.close()


wFile = open('algDump.txt', 'w+')



for i in range(count):
    outputStr = ''
    x = data[i].split()
    print(x)
    objLThigh.newValues([x[1], x[2], x[3], x[4], x[5], x[6], 0, 0, 0])
    objRThigh.newValues([x[7], x[8], x[9], x[10], x[11], x[12], 0, 0, 0])
    objLShank.newValues([x[13], x[14], x[15], x[16], x[17], x[18], 0, 0, 0])
    objRShank.newValues([x[19], x[20], x[21], x[22], x[23], x[24], 0, 0, 0])
    
    if i < 100:
        objRThigh.getCalib()
        objRShank.getCalib()

        objLThigh.getCalib()
        objLShank.getCalib()

    else:

        objRThigh.angleCalc()
        objRShank.angleCalc()

        objLThigh.angleCalc()
        objLShank.angleCalc()



    kneelingTorqueEstimationR, kneelingTorqueEstimationL, kneeAngleR, kneeAngleL, legForward = kneelingDetect.getTorque(objRThigh, objRShank, objLThigh, objLShank)
    outputStr = f"{kneelingTorqueEstimationL}\t{kneelingTorqueEstimationR}\t{kneeAngleL}\t{kneeAngleR}\t{legForward}\n"
    wFile.write(outputStr)
    time.sleep(.02)
wFile.close()
