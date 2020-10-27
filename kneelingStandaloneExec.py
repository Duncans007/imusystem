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
    objLThigh.newValues([int(x[1]), int(x[2]), int(x[3]), int(x[4]), int(x[5]), int(x[6]), 0, 0, 0])
    objRThigh.newValues([int(x[7]), int(x[8]), int(x[9]), int(x[10]), int(x[11]), int(x[12]), 0, 0, 0])
    objLShank.newValues([int(x[13]), int(x[14]), int(x[15]), int(x[16]), int(x[17]), int(x[18]), 0, 0, 0])
    objRShank.newValues([int(x[19]), int(x[20]), int(x[21]), int(x[22]), int(x[23]), int(x[24]), 0, 0, 0])
    
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
