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
        self.calibVal = .1  #How much gravity can affect angle during walking
        self.zAngle = 0
        
        self.gravAngleWindow = 0 #how high or low above the gravity vector does it have to be to get moved (still)
        self.standingCalibVal = .1 #how much can the gravity vector affect the angle (still)
        
        #angularAcceleration() variables
        self.gyZarray = [0]
        self.angularAcceleration = 0
        self.angularAccelerationMovingAvgAccuracy = 2
        
        #gravityVectorAngle() variables
        self.angleFromGravity = 0
        self.gravAngleArray = []
        self.gravAngleSmoothed = 0
        self.gravAngleArrayLimit = 5

#--------------------------------------------------------------------------------------------------
#Function to dump new values from sensors
#Function not currently in use b/c it's clunky to implement (the positive/negative values for each axis differ on each sensor, dictionary implementation should be possible when you go to implement it. This should cut 50+ lines out of the original file.)
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

    
#--------------------------------------------------------------------------------------------------    
#Angle Calcluation function        
    def angleCalc(self, gaitDetectObject):
        import time
        import numpy as np
        self.timeLastValue = self.currentTime
        self.currentTime = time.time()
        self.timeToRun = self.currentTime - self.timeLastValue
        
        self.gravityVectorAngle()
        self.angularAccCalc()
        
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
                if np.mean(self.zAngleArray) > self.gravAngleSmoothed:
                    self.zAngle -= self.calibVal
                elif np.mean(self.zAngleArray) < self.gravAngleSmoothed:
                    self.zAngle += self.calibVal
        
    #If standing still, reset angle to zero over time
        elif gaitDetectObject.standing == True:
            if self.zAngle > self.gravAngleSmoothed + self.gravAngleWindow:
                self.zAngle -= self.standingCalibVal
                self.zAngle += zAngleChange
            elif self.zAngle < self.gravAngleSmoothed - self.gravAngleWindow:
                self.zAngle += self.standingCalibVal
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
    
#--------------------------------------------------------------------------------------------------
#Gravity vector calculations function
    def gravityVectorAngle(self):
        import math
        import numpy as np
        
    #On all sensors gravity is depicted as approximately -9.81
        g = -9.81
    #Pythagorean theorem for acceleration vector magnitude (should be approximately -9.81 if not moving.)
        magnitude = ((self.acX ** 2) + (self.acY ** 2)) ** 0.5
        
        #Originally used to resize acceleration vector to be of the same magnitude as the gravity vector for easier comparison. Messes up extra while moving though.
        #ratio = abs(magnitude / g)
        
    #Apply dot product to system in 2 steps to find the angle between where gravity is and should be.
        dotProd = (g * self.acY) / (abs(g * magnitude))
        self.angleFromGravity = math.degrees(math.acos(dotProd))
        
    #ACOS function has limited output, direction of angle movement determined by the direction of X acceleration on the heel.
    #Negative X acc should imply toe up, while positive X acc should imply heel up. Values are originally only positive, so only necessary to change to negative when required.
    #This is not entirely accurate during walking, which is why the drift cap is set so low while walking. 
    #Note: this will not work at all as a primary method of angle detection. Works well to correct gyroscope values though.
        if self.acX > 0:
            self.angleFromGravity = -self.angleFromGravity
        
    #Populates and minimizes array of values to smooth curve. Usually set around 2-3 frames to minimize time delay while still smoothing peaks as much as possible.
    #2-3 frames adds up to .04-.06 seconds delay total, halved and averaged comes out to a .025 second (or approximately 1 frame) delay due to this method of smoothing.
        self.gravAngleArray.append(self.angleFromGravity)
        if len(self.gravAngleArray) > self.gravAngleArrayLimit:
            self.gravAngleArray.pop(0)
        
    #Take mean of populated array, save to object.
        self.gravAngleSmoothed = np.mean(self.gravAngleArray)
