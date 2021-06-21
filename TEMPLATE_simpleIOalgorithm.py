#!/usr/bin/env python3
#Name algorithm and define inputs
def algorithmName(alg_input_1, alg_input_2):
    
    #Define Constants
    constant_gravity = 9.81
    
    #Write calculations
    alg_output_1 = alg_input_1 + alg_input_2
    
    return alg_output_1

#Add algorithm to program:
# under "algorithms and secondary angle calculations", insert function call --> [ output = algorithmName(input) ]
#Add output to file:
# immediately before "header += f"\n"  fileDump.write(header)", add column title --> [ header += f"\tTITLE" ]
# under "DATA OUTPUT ----------", before "fileDump.write(f"{outputString}")", add output --> [ outputString += f"\t{output} ]
