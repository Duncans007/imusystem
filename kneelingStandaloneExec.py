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

mass = 80 #kg
NMKG = 0.15

objRThigh = sensorObject("RT")
objRShank = sensorObject("RS")
objLThigh = sensorObject("LT")
objLShank = sensorObject("LS")

kneelingDetect = kneelingDetection(NMKG, mass)

rFile = open('standaloneKneelingData.txt')
data = rFile.readlines()
count = len(data)
rFile.close()


wFile = open('algDump.txt', 'w+')



for i in range(count):
    outputStr = ''
    x = data[i].split()
    print(x)
    for enum, y in enumerate([objLThigh, objRThigh, objLShank, objRShank]):
        x.gyX = float(y[enum])
        x.gyY = float(y[enum + 1])
        x.gyZ = float(y[enum + 2])
        x.acX = float(y[enum + 3])
        x.acY = float(y[enum + 4])
        x.acZ = float(y[enum + 5])
        
    if i < 50:
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
    outputStr = f"{objLThigh.zAngle}\t{objRThigh.zAngle}\t{objLShank.zAngle}\t{objRShank.zAngle}\t{kneelingTorqueEstimationL}\t{kneelingTorqueEstimationR}\t{kneeAngleL}\t{kneeAngleR}\t{legForward}\n"
    wFile.write(outputStr)
    time.sleep(.02)
wFile.close()
