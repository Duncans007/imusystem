#!/usr/bin/env python3

class kneelingDetection:
    def __init__(self, NMKG, mass):
        import time
        self.NMKG = NMKG
        self.mass = mass
        
        #Inputs updated on the first loop
        self.thighAngleR = 0
        self.shankAngleR = 0
        
        self.thighAngleL = 0
        self.shankAngleL = 0
        
        self.thighRAngV  = 0
        self.shankRAngV  = 0
        
        self.thighLAngV  = 0
        self.shankLAngV  = 0
        
        self.kneeAngleR = 0
        self.kneeAngleL = 0
        
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
        self.timeLastKneeling = time.time()
        self.run_loop = False
        
        self.legForward = "X"
        self.lastLeg = "X"
    
    
    
    
    
    
    
    #Main function to run for third party input and export
    def getTorque(self, rThigh, rShank, lThigh, lShank):
        
        self.thighAngleR = rThigh.zAngle
        self.shankAngleR = rShank.zAngle
        
        self.thighAngleL = lThigh.zAngle
        self.shankAngleL = lShank.zAngle
        
        self.thighRAngV  = rThigh.gyZ
        self.shankRAngV  = rShank.gyZ
        
        self.thighLAngV  = lThigh.gyZ
        self.shankLAngV  = lShank.gyZ
        
        #Knee angles calculated with 0 as straight here
        self.kneeAngleR = 180 - (self.thighAngleR - self.shankAngleR)
        self.kneeAngleL = 180 - (self.thighAngleL - self.shankAngleL)
        
        self.kneelingDetection()
        
        if (self.legForward == "L"):
            self.lastLeg = "L"
        if (self.legForward == "R"):
            self.lastLeg = "R"
        if (self.legForward == "2"):
            self.lastLeg = "2"
        
        torqueL, torqueR = self.torqueEstimation(self.kneeAngleR, self.thighRAngV, self.kneeAngleL, self.thighLAngV)
            
        return torqueR, torqueL, self.kneeAngleR, self.kneeAngleL, self.legForward
        
        
    
    
    
    
    
    
    
    
    def torqueEstimation(self, kneeAngleR, thighGyR, kneeAngleL, thighGyL):
        #NMKG - Newton-meters per kilogram (for initial tests, 0.15, 0.30, 0.45)
        #mass - kilograms of subject
        #angVel and kneeAngle are for leg with device. angVel is for thigh.
        #Knee angles oriented with staight leg at 0 degrees
        
        
        if (self.torqueWindow("RIGHT")):
            torqueOutputR = (self.A * (180-kneeAngleR)) + (self.B * thighGyR) + self.C
            torqueOutputR = torqueOutputR * self.NMKG * self.mass * (12/15)
        else:
            torqueOutputR = 0
        
        
        if (self.torqueWindow("LEFT")):
            torqueOutputL = (self.A * (180-kneeAngleL)) + (self.B * thighGyL) + self.C
            torqueOutputL = torqueOutputL * self.NMKG * self.mass * (12/15)
        else:
            torqueOutputL = 0
        

        return torqueOutputL, torqueOutputR
    
    
    
    
    
    
    
    
    
    
    
    def torqueWindow(self, leg):
        #Knee angles oriented with staight leg at 180 degrees
        #leg = "RIGHT" or "LEFT"
        import time
        
        if (leg == "RIGHT"):
            localKneeAngle = self.kneeAngleR
            match = "R"
        
        if (leg == "LEFT"):
            localKneeAngle = self.kneeAngleL
            match = "L"
            
        if (self.legForward == "2"):
            localKneeAngle = (self.kneeAngleL + self.kneeAngleR) / 2
        
        #self.run_loop is a single-trip-switch, that shuts off torque after 
        if time.time() - self.timeLastKneeling < .6 and localKneeAngle < 170 and self.run_loop:
            self.run_loop = True
        else:
            self.run_loop = False
        
        
        #Condition 1: self.legForward == match
        c1 = (self.legForward == match)
        
        #Condition 2: self.legForward == "2"
        c2 = (self.legForward == "2")
        
        #Condition 3: self.legForward == X and lastLeg == match and single-trip-switch
        c3 = ((self.legForward == "X") and (self.lastLeg == match) and (self.run_loop))
        
        #Condition 4: self.legForward == X and lastLeg == "2" and single-trip-switch
        c4 = ((self.legForward == "X") and (self.lastLeg == 2) and (self.run_loop))
            
        if c1 or c2 or c3 or c4:
            deliverTorque = True
            self.run_loop = True
            if self.isKneeling == True:
                self.timeLastKneeling = time.time()
        else:
            deliverTorque = False
            
        return deliverTorque
        
        
        
        
        
        
        
        
        
        
        
        
        
    def kneelingDetection(self):
        #Knee angles oriented with staight leg at 180 degrees
        import numpy as np
        
        
        
    #Calculate mean and standard deviation of gyroscope data outside of if statements so that moving array is not compromised.
        self.movingAvgGyThighR.append(self.thighRAngV)
        self.movingAvgGyThighL.append(self.thighLAngV)

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
        
        R_thighR_shankL_angV = self.shankLAngV - self.thighRAngV
        L_thighL_shankR_angV = self.shankRAngV - self.thighLAngV
        
        
        
        
        
    #Implement early kneeling down detection via gyroscopes
        
        
        
        
        
        
        
    #Test if angle is past a rather large and easy to determine threshold (60 degrees from straight)
    #re-work to use sum of angles for a closer detection
        if (self.kneeAngleL < 120) and (self.kneeAngleR < 120):
            self.isKneeling = True
        else:
            self.isKneeling = False
            self.legForward = "X"
            

            
            
            
    #Test which foot is forward (or if both are backwards) using the angle of the shin to the horizontal.
    #Leg with horizontal shin is backwards, if both shins horizontal then both legs down.
    #To expand for kneeling on an angle, use the difference between the shin angles with a window for how close they can be, and the lesser/greater one is forward once it passes the threshold
        
        if self.isKneeling == True:
            legForwardThreshold = 30
            if abs(self.shankAngleR - self.shankAngleL) < legForwardThreshold:
                self.legForward = "2"
            #deep flexion test
                #if (rightKneeAngle < 60) and (leftKneeAngle < 60):
                    #self.legForward += "d"
            else:
                if self.shankAngleL > self.shankAngleR:
                    self.legForward = "L"
                    self.legWasForward = "L"
                elif self.shankAngleR > self.shankAngleL:
                    self.legForward = "R"
                    self.legWasForward = "R"
                    
                    
                    
                    
                    

#Detect a spike as the moment that the subject starts to stand up.
            if (self.thighRAngV < R_lower_limit) and (R_thighR_shankL_angV > R_upper_limit) and len(self.movingAvgGyThighR) > 20:
                #self.movingAvgGyThighR.pop(len(self.movingAvgGyThighR)-1)
                self.Rcounter = self.Rcounter + 1
            else:
                self.Rcounter = 0
                
            if (self.thighLAngV < L_lower_limit) and (L_thighL_shankR_angV > L_upper_limit) and len(self.movingAvgGyThighL) > 20:
                #self.movingAvgGyThighL.pop(len(self.movingAvgGyThighL)-1)
                self.Lcounter = self.Lcounter + 1
            else:
                self.Lcounter = 0
               
            
            
            
            
#Check for consecutive signals before setting to "standing up" mode.
            if (self.Rcounter >= self.counterDetectionLimit and self.legForward == "R") or (self.Lcounter >= self.counterDetectionLimit and self.legForward == "L"):
                self.startingToStand = True
            #((self.Rcounter >=1 and self.Lcounter >=1) and self.legForward == "2")
            
        if self.startingToStand == True:
            if (self.legWasForward == "R" and self.kneeAngleR > 160) or (self.legWasForward == "L" and self.kneeAngleL > 160):
                self.startingToStand = False
                self.legWasForward = "X"
            #self.legForward += "s"
