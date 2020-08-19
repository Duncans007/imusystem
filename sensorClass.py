class sensorObject:
    def __init__(self):
        import time
        #State Values from Sensors initialized at 0
        self.gyX = 0
        self.gyY = 0
        self.gyZ = 0
        
        self.acX = 0
        self.acY = 0
        self.acZ = 0
        
        self.mgX = 0
        self.mgY = 0
        self.mgZ = 0
        
        #angleCalc() variables
        self.timeToRun = 0
        self.timeLastValue = time.time()
        self.currentTime = time.time()
        
        self.zAngleChangeArray = [0]
        self.zAngleChangeArrayLimit = 5
        self.zAngleArray = [0]
        self.zAngleArrayLimit = 50
        self.wasStanding = False
        self.calibVal = .1
        self.zAngle = 0
        
        #angularAcceleration() variables
        self.gyZarray = [0]
        self.angularAcceleration = 0
        self.angularAccelerationMovingAvgAccuracy = 4

#Function not currently in use b/c 
    def newValues(self, valueArray):
        self.gyX = valueArray[0]
        self.gyY = valueArray[1]
        self.gyZ = valueArray[2]
        
        self.acX = valueArray[3]
        self.acY = valueArray[4]
        self.acZ = valueArray[5]
        
        self.mgX = valueArray[6]
        self.mgY = valueArray[7]
        self.mgZ = valueArray[8]
        
        
    def angularAccCalc(self):
        import time
        import numpy as np
        self.gyZarray.append(self.gyZ)
        
#Keep gyroscope array limited
        while len(self.gyZarray) > self.angularAccelerationMovingAvgAccuracy:
            self.gyZarray.pop(0)
            
#Take derivative of array
        diff_arr = np.diff(self.gyZarray)
        
#Averages the differences from diff() function.
        try:
            self.angularAcceleration = np.mean(diff_arr)
        except:
            self.angularAcceleration = 0
            
        return self.angularAcceleration

        
    def angleCalc(self, gaitDetectObject):
        import time
        import numpy as np
        self.timeLastValue = self.currentTime
        self.currentTime = time.time()
        self.timeToRun = self.currentTime - self.timeLastValue
        
#Integrates gyroscope to get angle. Includes drift.
        zAngleChange = self.gyZ * self.timeToRun
        
#Keeps track of previous values. Used when standing is turned off to make up for the 4-5 frame delay.
        self.zAngleChangeArray.append(zAngleChange)
        
#Limits number of previous values kept
        if len(self.zAngleChangeArray) > self.zAngleChangeArrayLimit:
            self.zAngleChangeArray.pop(0)
    
#When not standing
        if gaitDetectObject.standing == False:
        
        #If was standing on the last frame, add the missed values stored in zAngleChangeArray
            if self.wasStanding == True:
                self.wasStanding = False
                self.zAngle += np.sum(self.zAngleChangeArray)
        
        #Otherwise, add zAngleChange
            else:
                self.zAngle += zAngleChange
                
        #Drifts degree value towards 0 by 1/10th of a degree per frame.
                if np.mean(self.zAngleArray) > 0:
                    self.zAngle -= self.calibVal
                elif np.mean(self.zAngleArray) < 0:
                    self.zAngle += self.calibVal
        
    #If standing still, reset angle to zero over time
        elif gaitDetectObject.standing == True:
            if self.zAngle > 1:
                self.zAngle -= 1
                self.zAngle += zAngleChange
            elif self.zAngle < -1:
                self.zAngle += 1
                self.zAngle += zAngleChange
            else:
                pass
        #Tracks if standing was true on last frame for zAngleChangeArray
            self.wasStanding = True
            
    #Keeps short array of values. Not currently used.
        self.zAngleArray.append(self.zAngle)
    
        if len(self.zAngleArray) > self.zAngleArrayLimit:
            self.zAngleArray.pop(0)
            
        return self.zAngle
    
    
        def gravityVector():
            
