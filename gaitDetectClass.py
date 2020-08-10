#!/usr/bin/env python3

class gaitDetect:
    def __init__(self):       
        self.firstVar = 0
        self.movingArrShank = [0]
        self.movingArrHeel = [0]
        self.significance = 0
        self.movingAvgAccuracy = 2
        self.movingAvgShank = 0
        self.movingAvgHeel = 0
        self.lastAvgShank = 0
        self.timeLastHeelStrike = 0
        self.timeLastHeelOff = 0
        self.timeLastToeOff = 0
        self.gaitStage = 0 #0-Stance / 1-Heel Off / 2-Toe Off
        self.eventTimer = .1
        self.standing = False
        self.standingLimit = 200 * .07
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5
        self.lastDiffHeel = 0
        
                
    def testVal(self, shank, heel):
        self.movingArrShank.append(shank)
        self.movingArrHeel.append(heel)
        
        if len(self.movingArrShank) > self.movingAvgAccuracy:
            self.movingArrShank.pop(0)
        self.movingAvgShank = np.mean(self.movingArrShank)
        
        if len(self.movingArrHeel) > self.movingAvgAccuracy:
            self.movingArrHeel.pop(0)
        self.movingAvgHeel = np.mean(self.movingArrHeel)
        
        if self.standing == True:
            
            #Locks gait at 0 when standing detected, to prevent additional axes crossings. Recommended to leave on for live use.
            self.gaitStage = 0
            if self.movingAvgShank < - self.standingLimit or self.movingAvgShank > self.standingLimit:
                self.standing = False
                self.lastAvgShank = - self.movingAvgShank
                
        if self.standing == False:
            if self.movingAvgShank < self.standingLimit and self.movingAvgShank > - self.standingLimit:
                self.concurrentZeroes += 1
            else:
                self.concurrentZeroes = 0
                
            if self.concurrentZeroes > self.concurrentZeroesLimit:
                self.standing = True
        
        if self.significance == 0 and not self.standing:
            if self.movingAvgShank > 0 and self.lastAvgShank < 0 and self.gaitStage == 1: #detects negative to positive, aka toe off or start of swing phase
                self.significance = 1
                self.timeLastToeOff = time.time()
                self.gaitStage = 2
            elif self.movingAvgShank < 0 and self.lastAvgShank > 0 and self.gaitStage == 2: #detects positive to negative, aka heel strike or start of stance phase
                self.significance = -1
                self.timeLastHeelStrike = time.time()
                self.gaitStage = 0
            elif np.mean(np.diff(self.movingArrHeel)) < -20 and self.gaitStage == 0:
                if self.lastDiffHeel <= 20 and self.lastDiffHeel >= -20:
                    self.timeLastHeelOff = time.time()
                    self.significance = 2
                    self.gaitStage = 1

            self.lastDiffHeel = np.mean(np.diff(self.movingArrHeel))
                
        elif self.significance != 0:
            if time.time() - self.timeLastHeelStrike > self.eventTimer and time.time() - self.timeLastToeOff > self.eventTimer:
                self.significance = 0
            if self.significance == 2 and time.time() - self.timeLast:
                self.significance = 0
        
        #Implement other leg IMU - other leg heel strike must occur before measured leg toe off. (and vice versa)

        self.lastAvgShank = self.movingAvgShank
        
if __name__ == "__main__":
    rightLegGait = gaitDetect()
    gaitStages = []
    
    for enum, x in enumerate(inputGyZShank):
        rightLegGait.testVal(x * .07, inputGyZHeel[enum] * .07)
        gaitStages.append(rightLegGait.gaitStage)
        time.sleep(timeArray[enum])
        
    print(gaitStages)
