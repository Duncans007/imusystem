#!/usr/bin/env python3

#!/usr/bin/env python3

class gaitDetect:
    def __init__(self):
        import time
        import numpy as np
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
        self.timeLastStanding = 0
        self.gaitStage = 0 #0-Stance / 1-Heel Off / 2-Toe Off
        self.eventTimer = .1
        self.standing = False
        self.standingLimit = 200 * .07
        self.lastDiffHeel = 0
        self.gamma = 40 #-562 or -377 #deg/s	
        self.slipToeOffWaitThreshold = .2
        self.slipHeelStrikeWaitThreshold = .1
        self.indicatorThreshold = 10 ** 30
        self.isSlipping = False
        self.timeSlipStart = 0
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5


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
        
                
    def testVal(self, shank, heel):
        import time
        import numpy as np
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
                self.timeLastStanding = time.time()
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
                if self.lastDiffHeel <= 25 and self.lastDiffHeel >= -25:
                    self.timeLastHeelOff = time.time()
                    self.significance = 2
                    self.gaitStage = 1

            self.lastDiffHeel = np.mean(np.diff(self.movingArrHeel))
                
        elif self.significance != 0:
            if time.time() - self.timeLastHeelStrike > self.eventTimer and time.time() - self.timeLastToeOff > self.eventTimer:
                self.significance = 0
            if self.significance == 2:
                self.significance = 0
        
        #Implement other leg IMU - other leg heel strike must occur before measured leg toe off. (and vice versa)

        self.lastAvgShank = self.movingAvgShank
        
    def slipTrkov(self, pelvisAcc, forwardFootAcc, L_hh):
        import time
        if (self.gaitStage == 0 and time.time() - self.timeLastHeelStrike < self.slipHeelStrikeWaitThreshold) or (self.gaitStage == 2 and time.time() - self.timeLastToeOff > self.slipToeOffWaitThreshold):
            dd_q_hh = (pelvisAcc - forwardFootAcc) / L_hh
            slip_indicator = forwardFootAcc / (2.718 ** (dd_q_hh - self.gamma))
            return slip_indicator
        else:
            return 0



if __name__ == "__main__":
    import time
    rightLegGait = gaitDetect()
    gaitStages = []
    slipI = []
    
    for enum, x in enumerate(inputGyZShank):
        rightLegGait.testVal(x, inputGyZHeel[enum])
        slipI.append(rightLegGait.slipTrkov(pelvisAcc[enum], heelAcc[enum], 1))
        gaitStages.append(rightLegGait.gaitStage)
        print(gaitStages[enum])
        time.sleep(timeArray[enum])
        
    print(gaitStages)
    print(slipI)
