def package_handler_raw(tup):
    
    out = []
    
#Hard-coded numbers come from Chordata's signed integer conversion to standard values. More info can be found on the forums at forum.chordata.cc
#Additionally, usual Chordata output for accelerometer is in G, moved to m/s^2 for easier compatibility with Trkov IFAC 2017 slip detection algorithm.
    
    for pos, x in enumerate(tup):
        if pos < 3:
            #Gyroscope (degrees per second)
            out.append(x * .07)
        elif pos >=3 and pos < 6:
            #Accelerometer (m/s^2)
            out.append(x * .000244 * 9.81)
        elif pos >= 6:
            #Magnetometer (Gauss)
            out.append(x * .00014)
            
    return out
