def slipAlgorithm(pelvisAcc, forwardFootAcc, L_hh):
    #Update notes: double check that L_hh pulls from userinput.py

    global fileDump

    #Constants from paper Trkov "sensor based slip detection..." IEEE '19
    slip_constant = 2.83 #or 1.87
    beta = 2.718
    gamma = 40 #-562 or -377 #deg/s	
    
    dd_q_hh = (pelvisAcc - forwardFootAcc) / L_hh
    
    slip_indicator = forwardFootAcc / (beta ** (dd_q_hh - gamma))
    
    return slip_indicator
