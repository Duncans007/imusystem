#!/usr/bin/env python3

#Create object (must be run *once* before running function to create object)
class algorithmName:
    
    #Runs once on object creation
    def __init__(self, init_variable_1):
        
        #Define constants at object creation
        self.gravity = 9.81
        
        #Define input constants at object creation
        self.init_var = init_variable_1
        
        #Define any perpetual variables
        self.movingAvgArr = [0];
        
                
    def input_new_values_and_run_calculation(self, variable_input_1):
        #Example: Moving Average calculation
        
        #Add function input to array
        self.movingAvgArr.append(variable_input_1)
        
        #Keep array size limited
        if len(self.movingAvgArr) > 10:
            self.movingAvgArr.pop(0)
        
        #Return moving average
        return mean(self.movingAvgArr)
    
    
#To insert into main code (file variableSensorMain.py):
#(4) line insertions in total:
# under "Setup">"Variable initializations", insert object creation --> [ objectName = algorithmName(init_variables) ]
# under "algorithms and secondary angle calculations", insert function call --> [ output = objectName.input_new_values_and_run_calculation(input) ]
# under "Setup">"Create formatted file header" immediately before "header += f"\n"  fileDump.write(header)", add column title --> [ header += f"\tTITLE" ]
# under "DATA OUTPUT", before "fileDump.write(f"{outputString}")", add output --> [ outputString += f"\t{output} ]
