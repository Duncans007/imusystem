#!/usr/bin/env python3

class kneelingDetection:
    def __init__(self):
        #Perpetual variables for kneelingDetection()
        self.movingAvgLen = 50
        self.movingAvgGyThighR = []
        self.movingAvgGyThighL = []
        self.Rcounter = 0
        self.Lcounter = 0
        self.isKneeling = False
        self.stdMultiplier = 2
        self.counterDetectionLimit = 2
        self.startingToStand = False
        self.legWasForward = "X"
        
        #Perpetual values for torqueEstimation()
        self.A = 0.012
        self.B = -0.002
        self.C = -0.075
        
        #torqueWindow()
        self.deliverTorque = False
        self.timeLastKneeling = 0

    def torqueEstimation(self, NMKG, mass, kneeAngle, angVel):
        #NMKG - Newton-meters per kilogram (for initial tests, 0.15, 0.30, 0.45)
        #mass - kilograms of subject
        #angVel and kneeAngle are for leg with device. angVel is for thigh.
        
        kneeAngleCorrected = 180 - kneeAngle
        torqueOutput = (self.A * kneeAngleCorrected) + (self.B * angVel) + self.C
        torqueOutput = torqueOutput * NMKG * mass * (12/15)
        
        if self.torqueWindow(kneeAngle):
            return torqueOutput
        else:
            return 0
    
    def torqueWindow(self, kneeAngle):
        import time
        if self.isKneeling == True or (time.time() - self.timeLastKneeling < .5 and kneeAngle < 170):
            self.deliverTorque = True
            if self.isKneeling == True:
                self.timeLastKneeling = time.time()
        else:
            self.deliverTorque = False
            
        return self.deliverTorque
        
    def kneelingDetection(self, thighObjR, shankObjR, heelObjR, thighObjL, shankObjL, heelObjL):
        import numpy as np
    #Pull angle measurements for full state.
    #Might make this its own file in the future
    #Simply because it uses measurements from both legs at the same time, which is not the purpose of the gaitDetect class
    #Although the gait detect class can definitely be repurposed to take all seven values and calculate for both legs simultaneously.
        thighAngleR = thighObjR.zAngle
        shankAngleR = shankObjR.zAngle
        heelAngleR = heelObjR.zAngle
        
        thighAngleL = thighObjL.zAngle
        shankAngleL = shankObjL.zAngle
        heelAngleL = heelObjL.zAngle
        
        thighLAngV = thighObjL.gyZ
        thighRAngV = thighObjR.gyZ
        
        shankLAngV = shankObjL.gyZ
        shankRAngV = shankObjR.gyZ
        
        kneelingGyLimit = 60
        
        legForward = ""
        
    #Calculate knee angle of both legs, with 180 being standing straight and 90 being bent halfway
        leftKneeAngle = 180 - abs(thighAngleL - shankAngleL)
        rightKneeAngle = 180 - abs(thighAngleR - shankAngleR)
        
    #Calculate mean and standard deviation of gyroscope data outside of if statements so that moving array is not compromised.
        self.movingAvgGyThighR.append(thighRAngV)
        self.movingAvgGyThighL.append(thighLAngV)

        if len(self.movingAvgGyThighR) > self.movingAvgLen:
            self.movingAvgGyThighR.pop(0)
        if len(self.movingAvgGyThighL) > self.movingAvgLen:
            self.movingAvgGyThighL.pop(0)

        Rmean = np.mean(self.movingAvgGyThighR)
        Rsd = np.std(self.movingAvgGyThighR) * self.stdMultiplier
            
        Lmean = np.mean(self.movingAvgGyThighL)
        Lsd = np.std(self.movingAvgGyThighL) * self.stdMultiplier
        
        if Rsd < 5:
            Rsd = Rsd * 2
        if Lsd < 5:
            Lsd = Lsd * 2
            
        R_upper_limit = Rmean + Rsd
        R_lower_limit = Rmean - Rsd
        
        L_upper_limit = Lmean + Lsd
        L_lower_limit = Lmean - Lsd
        
        R_thighR_shankL_angV = shankLAngV - thighRAngV
        L_thighL_shankR_angV = shankRAngV - thighLAngV
        
    #Test if angle is past a rather large and easy to determine threshold (60 degrees from straight)
        if (leftKneeAngle < 120) and (rightKneeAngle < 120):
            self.isKneeling = True
        else:
            self.isKneeling = False
            legForward = "X"
            
            
    #Test which foot is forward (or if both are backwards) using the angle of the shin to the horizontal.
    #Leg with horizontal shin is backwards, if both shins horizontal then both legs down.
    #To expand for kneeling on an angle, use the difference between the shin angles with a window for how close they can be, and the lesser/greater one is forward once it passes the threshold
        
        if self.isKneeling == True:
            legForwardThreshold = 30
            if abs(shankAngleR - shankAngleL) < legForwardThreshold:
                legForward = "2"
            #deep flexion test
                #if (rightKneeAngle < 60) and (leftKneeAngle < 60):
                    #legForward += "d"
            else:
                if shankAngleL > shankAngleR:
                    legForward = "L"
                    self.legWasForward = "L"
                elif shankAngleR > shankAngleL:
                    legForward = "R"
                    self.legWasForward = "R"
                    

#Detect a spike as the moment that the subject starts to stand up.
            if (thighRAngV < R_lower_limit) and (R_thighR_shankL_angV > R_upper_limit) and len(self.movingAvgGyThighR) > 20:
                #self.movingAvgGyThighR.pop(len(self.movingAvgGyThighR)-1)
                self.Rcounter = self.Rcounter + 1
            else:
                self.Rcounter = 0
                
            if (thighLAngV < L_lower_limit) and (L_thighL_shankR_angV > L_upper_limit) and len(self.movingAvgGyThighL) > 20:
                #self.movingAvgGyThighL.pop(len(self.movingAvgGyThighL)-1)
                self.Lcounter = self.Lcounter + 1
            else:
                self.Lcounter = 0
               
#Check for consecutive signals before setting to "standing up" mode.
            if (self.Rcounter >= self.counterDetectionLimit and legForward == "R") or (self.Lcounter >= self.counterDetectionLimit and legForward == "L"):
                self.startingToStand = True
            #((self.Rcounter >=1 and self.Lcounter >=1) and legForward == "2")
            
        if self.startingToStand == True:
            if (self.legWasForward == "R" and rightKneeAngle > 160) or (self.legWasForward == "L" and leftKneeAngle > 160):
                self.startingToStand = False
                self.legWasForward = "X"
            legForward += "s"
            
        return legForward, rightKneeAngle, leftKneeAngle, R_upper_limit, R_lower_limit, R_thighR_shankL_angV
