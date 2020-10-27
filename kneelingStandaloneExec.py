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
    objLThigh.newValues([float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4]), float(x[5]), 0, 0, 0])
    objRThigh.newValues([float(x[6]), float(x[7]), float(x[8]), float(x[9]), float(x[10]), float(x[11]), 0, 0, 0])
    objLShank.newValues([float(x[12]), float(x[13]), float(x[14]), float(x[15]), float(x[16]), float(x[17]), 0, 0, 0])
    objRShank.newValues([float(x[18]), float(x[19]), float(x[20]), float(x[21]), float(x[22]), float(x[23]), 0, 0, 0])
    
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
