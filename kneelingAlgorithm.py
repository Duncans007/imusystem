#!/usr/bin/env python3

class kneelingDetection:
    def __init__(self):
        self.movingAvgLen = 0
        self.movingAvgGyThighR = []
        self.movingAvgGyThighL = []
        self.Rcounter = 0
        self.Lcounter = 0

        
    def kneelingDetection(self, thighObjR, shankObjR, heelObjR, thighObjL, shankObjL, heelObjL):
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
        
        kneelingGyLimit = 60
        
        legForward = ""
        
    #Calculate knee angle of both legs, with 180 being standing straight and 90 being bent halfway
        leftKneeAngle = 180 - abs(thighAngleL - shankAngleL)
        rightKneeAngle = 180 - abs(thighAngleR - shankAngleR)
        
    #Test if angle is past a rather large and easy to determine threshold
        if (leftKneeAngle < 120) and (rightKneeAngle < 120):
            isKneeling = True
        else:
            isKneeling = False
            legForward = "X"
            
            
    #Test which foot is forward (or if both are backwards) using the angle of the shin to the horizontal.
    #Leg with horizontal shin is backwards, if both shins horizontal then both legs down.
    #To expand for kneeling on an angle, use the difference between the shin angles with a window for how close they can be, and the lesser/greater one is forward once it passes the threshold
        
        if isKneeling == True:
            legForwardThreshold = 30
            if abs(shankAngleR - shankAngleL) < legForwardThreshold:
                legForward = "2"
            else:
                if shankAngleL > shankAngleR:
                    legForward = "L"
                elif shankAngleR > shankAngleL:
                    legForward = "R"

#Detect a spike as the moment that the subject starts to stand up.
            self.movingAvgGyThighR.append(thighRAngV)
            self.movingAvgGyThighL.append(thighLAngV)

            if len(self.movingAvgGyThighR) > self.movingAvgLen:
                self.movingAvgGyThighR.pop(0)
            if len(self.movingAvgGyThighL) > self.movingAvgLen:
                self.movingAvgGyThighL.pop(0)

            Rmean = np.mean(self.movingAvgGyThighR)
            Rsd = np.std(self.movingAvgGyThighR) * 4
            
            Lmean = np.mean(self.movingAvgGyThighL)
            Lsd = np.std(self.movingAvgGyThighL) * 4
  
            if ((thighRAngV > Rmean + Rsd) or (thighRAngV < Rmean - Rsd)) and len(self.movingAvgGyThighR) > 20:
                self.movingAvgGyThighR.pop(len(self.movingAvgGyThighR)-1)
                self.Rcounter = self.Rcounter + 1
            else:
                self.Rcounter = 0
                
            if ((thighLAngV > Lmean + Lsd) or (thighLAngV < Lmean - Lsd)) and len(self.movingAvgGyThighL) > 20:
                self.movingAvgGyThighL.pop(len(self.movingAvgGyThighL)-1)
                self.Lcounter = self.Lcounter + 1
            else:
                self.Lcounter = 0
                
            if (Rcounter >= 2 and legForward == "R") or (Lcounter >= 2 and legForward == "L"):
                legForward += "s"


            #if (thighLAngV < - kneelingGyLimit and legForward == "L"):
            #    legForward += "s"
            #if (thighRAngV < - kneelingGyLimit and legForward == "R"):
            #    legForward += "s"
            
        return legForward, rightKneeAngle, leftKneeAngle
