# This file contains a class for storing and processing the values in an individual sensor.
# The class takes in unprocessed, unsigned ints from the Notochord and converts them to understandable units.
# It contains functions for calibrating based on sensor inputs and automatically applies the calibration values, however the calibration functions must be specified from the main file.
# It uses a filter (originally complimentary, updated to kalman) filter to determine sensor angles in 2 dimensions (pitch and roll)

import numpy as np

class sensorObject:
    def __init__(self, limbCode):
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

        self.zAngle = 0
        self.zAngleZeroed = 0

        self.xAngle = 0
        self.xAngleZeroed = 0
        
        #angularAcceleration() variables
        self.gyZarray = [0]
        self.angularAcceleration = 0
        self.angularAccelerationMovingAvgAccuracy = 2


        #angleCalc with kalman filter
        self.lastRunTime = 0
        self.Q_coeff = 0.0000001
        self.Q = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]]) * self.Q_coeff

        self.R_coeff = 0.001
        self.R = np.array([[1, 0],
                           [0, 1]]) * self.R_coeff

        self.C = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])

        self.x = np.transpose(np.array([[0, 0, 0, 0]]))

        self.P = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        self.acc_x = [0, 0]
        self.acc_y = [0, 0]
        self.acc_z = [0, 0]

        self.acc_angle_roll = [0]
        self.acc_angle_pitch = [0]
        self.roll_dot_arr = [0]
        self.pitch_dot_arr = [0]

        self.acc_roll_off = 0
        self.roll_calib_arr = []
        self.acc_pitch_off = 0
        self.pitch_calib_arr = []

        self.pitch_calib_val = 0
        self.roll_calib_val = 0

        
        
        
        
        
        
#--------------------------------------------------------------------------------------------------


#Function to dump new values from sensors
    def newValues(self, valueArray):
        outArray = []
        
        if self.limbCode == "RT" or self.limbCode == "RS":
            outArray = valueArray
            
        if self.limbCode == "RH":
            outArray = (-valueArray[1], valueArray[0], valueArray[2], -valueArray[4], valueArray[3], valueArray[5], -valueArray[7], valueArray[6], valueArray[8])
            
        if self.limbCode == "LT" or self.limbCode == "LS":
            outArray = (-valueArray[0], valueArray[1], -valueArray[2], -valueArray[3], valueArray[4], -valueArray[5], -valueArray[6], valueArray[7], -valueArray[8])
            
        if self.limbCode == "LH":
            outArray = (-valueArray[1], -valueArray[0], -valueArray[2], -valueArray[4], -valueArray[3], -valueArray[5], -valueArray[7], -valueArray[6], -valueArray[8])
            
        if self.limbCode == "LB":
            outArray = (-valueArray[2], valueArray[1], -valueArray[0], -valueArray[5], valueArray[4], -valueArray[3], -valueArray[8], valueArray[7], -valueArray[6])
            
            
            
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
        self.angleCalc()

        if len(self.pitch_calib_arr) < 100:
            self.pitch_calib_arr.append(self.zAngle)
            self.roll_calib_arr.append(self.xAngle)
        elif len(self.pitch_calib_arr) == 100:
            self.pitch_calib_arr.append(self.zAngle)
            self.roll_calib_arr.append(self.xAngle)
            self.pitch_calib_val = np.mean( self.pitch_calib_arr )
            self.roll_calib_val = np.mean( self.roll_calib_arr )
        else:
            self.zAngleZeroed -= self.pitch_calib_val
            self.xAngleZeroed -= self.roll_calib_val

#--------------------------------------------------------------------------------------------------
    def angularAccCalc(self):
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




    def angleCalc(self):
        # Used to update state of object with new values
        # Returns roll and pitch and updates system
        # Unpack inputs
        self.acc_x.append(self.acX)
        self.acc_y.append(self.acY)
        self.acc_z.append(self.acZ)
        self.acc_x.pop(0)
        self.acc_y.pop(0)
        self.acc_z.pop(0)
        acc_x_mean = np.mean(self.acc_x)
        acc_y_mean = np.mean(self.acc_y)
        acc_z_mean = np.mean(self.acc_z)

        gyro_x = np.radians(self.gyX)
        gyro_y = np.radians(self.gyY)
        gyro_z = np.radians(self.gyZ)
        roll_current = self.x[0][0]
        pitch_current = self.x[2][0]

        # Get adn update time since last calculation
        dt = time.time() - self.lastRunTime
        self.lastRunTime = time.time()

        # Calculate accelerometer angles
        accel_roll = -np.arctan2(acc_z_mean, np.sqrt((acc_y_mean * acc_y_mean) + (acc_x_mean * acc_x_mean)))
        accel_pitch = np.arctan2(-acc_x_mean, np.sqrt((acc_z_mean * acc_z_mean) + (acc_y_mean * acc_y_mean)))

        self.acc_angle_roll.append(np.degrees(accel_roll))
        self.acc_angle_pitch.append(np.degrees(accel_pitch))

        # Get euler angle derivatives of input gyroscope values
        roll_dot = gyro_x \
                   + (np.sin(roll_current) * np.tan(pitch_current) * gyro_z) \
                   + (np.cos(roll_current) * np.tan(pitch_current) * gyro_y)
        pitch_dot = np.cos(roll_current) * gyro_z - np.sin(roll_current) * gyro_y

        self.roll_dot_arr.append(np.degrees(roll_dot))
        self.pitch_dot_arr.append(np.degrees(pitch_dot))

        # Calculate A and B arrays based on time step
        A = np.array([[1, -dt, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, -dt],
                      [0, 0, 0, 1]])

        B = np.array([[dt, 0],
                      [0, 0],
                      [0, dt],
                      [0, 0]])

        # Assemble input matrices
        measured_input = np.transpose(np.array([[roll_dot, pitch_dot]]))
        acceleration_estimates = np.transpose(np.array([[accel_roll, accel_pitch]]))

        # Prediction equations
        x_new = (A.dot(self.x)) + (B.dot(measured_input))
        self.P = A.dot(self.P.dot(np.transpose(A))) + self.Q

        # Update equations
        y_new = acceleration_estimates - self.C.dot(x_new)
        S = self.C.dot(self.P.dot(np.transpose(self.C))) + self.R
        K = self.P.dot(np.transpose(self.C).dot(np.linalg.inv(S)))
        x_new = x_new + K.dot(y_new)

        self.x = x_new
        self.P = (np.eye(4) - K.dot(self.C)).dot(self.P)

        self.zAngle = np.degrees(self.x[2][0])
        self.zAngleZeroed = np.degrees(self.x[2][0])

        self.xAngle = np.degrees(self.x[0][0])
        self.xAngleZeroed = np.degrees(self.x[0][0])

#----------------------------------- ---------------------------------------------------------------



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
        self.gyY = self.gyY_norm * self.gyConversion
        self.gyZ = self.gyZ_norm * self.gyConversion
        
        self.acX = self.acX * self.acConversion
        self.acY = self.acY * self.acConversion
        self.acZ = self.acZ * self.acConversion
        
        self.mgX = self.mgX * self.mgConversion
        self.mgY = self.mgY * self.mgConversion
        self.mgZ = self.mgZ * self.mgConversion
        
        
