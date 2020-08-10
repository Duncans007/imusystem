class sensorObject:
    def __init(self):
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
    
    def newValues(self, valueArray):
        self.gyX = valueArray[0]
        self.gyY = valueArray[0]
        self.gyZ = valueArray[0]
        
        self.acX = valueArray[0]
        self.acY = valueArray[0]
        self.acZ = valueArray[0]
        
        self.mgX = valueArray[0]
        self.mgY = valueArray[0]
        self.mgZ = valueArray[0]
        
        
    def angularAccCalc(self):
        self.gyZarray.append(self.gyZ)
        
        while len(self.gyZarray) > self.angularAccelerationMovingAvgAccuracy:
            self.gyZarray.pop(0)
            
        diff_arr = np.diff(self.gyZarray)
        
        try:
            self.angularAcceleration = np.mean(diff_arr)
        except:
            self.angularAcceleration = 0
            
        return self.angularAcceleration

        
    def angleCalc(self, gaitDetectObject):
        self.timeLastValue = self.currentTime
        self.currentTime = time.time()
        self.timeToRun = self.currentTime - self.timeLastValue
        
        zAngleChange = self.gyZ * self.timeToRun
        
        self.zAngleChangeArray.append(zAngleChange)
        
        if len(self.zAngleChangeArray) > self.zAngleChangeArrayLimit:
            self.zAngleChangeArray.pop(0)
    
        if gaitDetectObject.standing == False:
            if self.wasStanding == True:
                self.wasStanding = False
                self.zAngle += np.sum(self.zAngleChangeArray)
                self.calibVal = .1
            else:
                self.zAngle += zAngleChange
                #self.calibVal += .001
                if np.mean(self.zAngleArray) > 0:
                    self.zAngle -= self.calibVal
                elif np.mean(self.zAngleArray) < 0:
                    self.zAngle += self.calibVal
                
        elif gaitDetectObject.standing == True:
            if self.zAngle > 1:
                self.zAngle -= 1
            elif self.zAngle < -1:
                self.zAngle += 1
            else:
                pass
            self.wasStanding = True
            
        
        self.zAngleArray.append(self.zAngle)
    
        if len(self.zAngleArray) > self.zAngleArrayLimit:
            self.zAngleArray.pop(0)
            
        return self.zAngle
