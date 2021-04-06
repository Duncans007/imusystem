#!/usr/bin/env python3

class gaitDetect:
    def __init__(self):
        self.firstVar = 0
        self.movingArrShank = [0]
        self.movingArrThigh = [0]
        self.movingArrHeel = [0]
        self.movingAvgShank = 0
        self.movingAvgThigh = 0
        self.movingAvgHeel = 0
        self.lastAvgShank = 0
        self.lastAvgThigh = 0
        self.movingAvgAccuracy = 2
        
        self.significance = 0
        self.gaitStage = 0 #0-Stance / 1-Heel Off / 2-Toe Off
        self.eventTimer = .1
        
        self.timeLastHeelStrike = 0
        self.timeLastHeelOff = 0
        self.timeLastToeOff = 0
        self.timeLastStanding = 0
        
        self.slipToeOffWaitThreshold = .2
        self.slipHeelStrikeWaitThreshold = .1
        self.standing = False
        self.standingLimit = 14 #deg/s
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5
        
        self.lastDiffHeel = 0
        self.gamma = 40 #-562 or -377 #deg/s	
        self.indicatorThreshold = 10 ** 30
        self.isSlipping = False
        self.timeSlipStart = 0
        
                
    def testVal(self, thigh, shank, heel):
        import time
        import numpy as np
        self.movingArrThigh.append(thigh)
        self.movingArrShank.append(shank)
        self.movingArrHeel.append(heel)
        
#Limits number of previous values in array
        if len(self.movingArrShank) > self.movingAvgAccuracy:
            self.movingArrShank.pop(0)
        self.movingAvgShank = np.mean(self.movingArrShank)
        
        if len(self.movingArrHeel) > self.movingAvgAccuracy:
            self.movingArrHeel.pop(0)
        self.movingAvgHeel = np.mean(self.movingArrHeel)
        
        if len(self.movingArrThigh) > self.movingAvgAccuracy:
            self.movingArrThigh.pop(0)
        self.movingAvgThigh = np.mean(self.movingArrThigh)
        

#Significance changes for timing reasons. When not zero, it waits for the given timeframe before resetting to zero so that gait change can be detected again
        if self.significance == 0:

#detects negative to positive, aka toe off or start of swing phase
            if self.movingAvgThigh > 0 and self.lastAvgThigh < 0 and self.gaitStage == 1:
                self.significance = 1
                self.timeLastToeOff = time.time()
                self.gaitStage = 2
                
#detects positive to negative, aka heel strike or start of stance phase
            elif self.movingAvgShank < 0 and self.lastAvgShank > 0 and self.gaitStage == 2: 
                self.significance = -1
                self.timeLastHeelStrike = time.time()
                self.gaitStage = 0

#detects heel off occurrence
            elif np.mean(np.diff(self.movingArrHeel)) < -20 and self.gaitStage == 0:
                if self.lastDiffHeel <= 25 and self.lastDiffHeel >= -25:
                    self.timeLastHeelOff = time.time()
                    self.significance = 2
                    self.gaitStage = 1
            #Keep records of previous values for checking at the beginning of function
            self.lastDiffHeel = np.mean(np.diff(self.movingArrHeel))
         
#When significance has been changed, wait the appropriate amount of time before resetting to allow for next gait update
        elif self.significance != 0:
            if time.time() - self.timeLastHeelStrike > self.eventTimer and time.time() - self.timeLastToeOff > self.eventTimer:
                self.significance = 0
            if self.significance == 2:
                self.significance = 0

        self.lastAvgShank = self.movingAvgShank
        self.lastAvgThigh = self.movingAvgThigh
        if (self.gaitStage == 2):
            self.gaitOutput = 1
        else:
            self.gaitOutput = 0
        
#Trkov IFAC 2017 slip detection algorithm
    def slipTrkov(self, pelvisAcc, forwardFootAcc, L_hh):
        import time
        
#Limits the window of slip detection to [slipToeOffWaitThreshold seconds after toe off -- heel strike]. Note that gait does not reset until after slip is done.
        if (self.gaitStage == 0 and time.time() - self.timeLastHeelStrike < self.slipHeelStrikeWaitThreshold) or (self.gaitStage == 2 and time.time() - self.timeLastToeOff > self.slipToeOffWaitThreshold):
            dd_q_hh = (pelvisAcc - forwardFootAcc) / L_hh
            slip_indicator = forwardFootAcc / (2.718 ** (dd_q_hh - self.gamma))
            return slip_indicator
        else:
            return 0

        
        
        

#For testing purposes
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
