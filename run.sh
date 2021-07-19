#!/bin/bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT

#echo "Arg1: NUC, Arg2: Teensy, Arg3: VICON. Both arguments are true/false all lowercase."

#sleep 1

(cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=97 -y localhost)&

sleep 6

./variableSensorMain.py #$1 $2 $3 #>/dev/null
