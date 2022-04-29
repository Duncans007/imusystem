#!/usr/bin/env python3

# This file initializes a number of variables that were needlessly bloating the main file.


packetReady = False
rPacketReady = False
packetWasReady = True

ip = "localhost"
port = 6565

dataDict = {
    "rThigh":  [0,0,0,0,0,0,0,0,0],
    "rShank":  [0,0,0,0,0,0,0,0,0],
    "rHeel":  [0,0,0,0,0,0,0,0,0],
    "lThigh": [0,0,0,0,0,0,0,0,0],
    "lShank": [0,0,0,0,0,0,0,0,0],
    "lHeel": [0,0,0,0,0,0,0,0,0],
    "lowBack":  [0,0,0,0,0,0,0,0,0],
    "topBack": [0,0,0,0,0,0,0,0,0]
}


cuny_data = {
    "ActTqL" : 0,
    "ActTqR" : 0,
    "KneeL"  : 0,
    "KneeR"  : 0,
    "LTangX" : 0,
    "LTangY" : 0,
    "LTangZ" : 0,
    "LTgyX"  : 0,
    "LTgyY"  : 0,
    "LTgyZ"  : 0,
    "LTacX"  : 0,
    "LTacY"  : 0,
    "LTacZ"  : 0,
    "RTangX" : 0,
    "RTangY" : 0,
    "RTangZ" : 0,
    "RTgyX"  : 0,
    "RTgyY"  : 0,
    "RTgyZ"  : 0,
    "RTacX"  : 0,
    "RTacY"  : 0,
    "RTacZ"  : 0,
    "LSangX" : 0,
    "LSangY" : 0,
    "LSangZ" : 0,
    "LSgyX"  : 0,
    "LSgyY"  : 0,
    "LSgyZ"  : 0,
    "LSacX"  : 0,
    "LSacY"  : 0,
    "LSacZ"  : 0,
    "RSangX" : 0,
    "RSangY" : 0,
    "RSangZ" : 0,
    "RSgyX"  : 0,
    "RSgyY"  : 0,
    "RSgyZ"  : 0,
    "RSacX"  : 0,
    "RSacY"  : 0,
    "RSacZ"  : 0,
    "BangX"  : 0,
    "BangY"  : 0,
    "BangZ"  : 0,
    "BgyX"   : 0,
    "BgyY"   : 0,
    "BgyZ"   : 0,
    "BacX"   : 0,
    "BacY"   : 0,
    "BacZ"   : 0
}


nuc_data = {
    "L"  : 0,
    "R"  : 0,
    "B"   : 0
}


flagDict = {
    "rThigh": False,
    "rShank": False,
    "rHeel": False,
    "lThigh": False,
    "lShank": False,
    "lHeel": False,
    "lowBack": False,
    "topBack": False
}


addressDict = {
    "10": "rThigh",
    "11": "rShank",
    "12": "rHeel",
    "30": "lThigh",
    "31": "lShank",
    "32": "lHeel",
    "20": "lowBack",
    "52": "topBack"
}


orderDict = {
    0: "rThigh",
    1: "rShank",
    2: "rHeel",
    3: "lThigh",
    4: "lShank",
    5: "lHeel",
    6: "lowBack",
    7: "topBack"
}


passToAlgorithm = {
    "rt_raw": [],
    "rs_raw": [],
    "rh_raw": [],
    "lt_raw": [],
    "ls_raw": [],
    "lh_raw": [],
    "b_raw": [],
    "tb_raw": []
}
