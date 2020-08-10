def package_handler_raw(tup):
    
    out = []
    
    for pos, x in enumerate(tup):
        if pos < 3:
            out.append(x * .07)
        elif pos >=3 and pos < 6:
            out.append(x * .000244 * 9.81)
        elif pos >= 6:
            out.append(x * .00014)
            
    return out
