import numpy as np

class sensorObject:
    def __init__(self, limbCode):
        import time
        self.limbCode = limbCode
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
        
        self.gyX_norm = 0
        self.gyY_norm = 0
        self.gyZ_norm = 0
        
        self.acX_norm = 0
        self.acY_norm = 0
        self.acZ_norm = 0
        
        self.mgX_norm = 0
        self.mgY_norm = 0
        self.mgZ_norm = 0
        
        self.gyConversion = 0.07
        self.acConversion = 0.000244 * 9.81
        self.mgConversion = 0.00014
        
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
    def newValues(self, valueArray):
        valueArray = np.array(valueArray)
        outArray = []
        
        if self.limbCode == "RT" or self.limbCode == "RS":
            outArray = valueArray
        if self.limbCode == "RH":
            outArray = [-valueArray[1], valueArray[0], valueArray[2], -valueArray[4], valueArray[3], valueArray[5], -valueArray[7], valueArray[6], valueArray[8]]
        if self.limbCode == "LT" or self.limbCode == "LS":
            outArray = np.multiply(valueArray, np.array([-1, 1, -1, -1, 1, -1, -1, 1, -1]))
        if self.limbCode == "LH":
            outArray = [-valueArray[1], -valueArray[0], -valueArray[2], -valueArray[4], -valueArray[3], -valueArray[5], -valueArray[7], -valueArray[6], -valueArray[8]]
        if self.limbCode == "LB":
            outArray = [-valueArray[2], valueArray[1], valueArray[0], -valueArray[5], valueArray[4], valueArray[3], -valueArray[8], valueArray[7], valueArray[6]]
        
        self.gyX = outArray[0]
        self.gyY = outArray[1]
        self.gyZ = outArray[2]
        
        self.acX = outArray[3]
        self.acY = outArray[4]
        self.acZ = outArray[5]
        
        self.mgX = outArray[6]
        self.mgY = outArray[7]
        self.mgZ = outArray[8]
        
        self.conversions()
        
        
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
    def angleCalc(self):
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
            
    #manually set perturbation range for now, later set using calibration function
        if (self.gyZ < self.gyZrange and self.gyZ > - self.gyZrange):
            if (self.gyY < self.gyYrange and self.gyY > - self.gyYrange):
                if (self.gyX < self.gyXrange and self.gyX > - self.gyXrange):
                    proportionality = abs(self.gravAngleSmoothed - - self.zAngle) / 20
                    if self.zAngle > self.gravAngleSmoothed + self.gravAngleWindow:
                        self.zAngle -= proportionality
                    elif self.zAngle < self.gravAngleSmoothed - self.gravAngleWindow:
                        self.zAngle += proportionality
        
        self.zAngle += zAngleChange
        
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
        
        
#--------------------------------------------------------------------------------------------------
#Function to streamline implementation of Alborz's new communication protocol while changing minimal code.
#This object now has the original signed 16-bit integers from the Notochord added directly to it.
#This function serves to put the 16-bit integers back to their more floated values with units for calculation.
#Runs at the beginning of testval, before any other calculations are done with the numbers.
    def conversions(self):
        self.gyX_norm = self.gyX
        self.gyY_norm = self.gyY
        self.gyZ_norm = self.gyZ
        
        self.acX_norm = self.acX
        self.acY_norm = self.acY
        self.acZ_norm = self.acZ
        
        self.mgX_norm = self.mgX
        self.mgY_norm = self.mgY
        self.mgZ_norm = self.mgZ
        
        self.gyX = self.gyX_norm * self.gyConversion
        self.gyY = self.gyX_norm * self.gyConversion
        self.gyZ = self.gyX_norm * self.gyConversion
        
        self.acX = self.acX * self.acConversion
        self.acY = self.acY * self.acConversion
        self.acZ = self.acZ * self.acConversion
        
        self.mgX = self.mgX * self.mgConversion
        self.mgY = self.mgY * self.mgConversion
        self.mgZ = self.mgZ * self.mgConversion
        
        
