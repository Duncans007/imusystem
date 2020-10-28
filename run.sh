#!/bin/bash

echo "Arg1: NUC, Arg2: Teensy. Both arguments are true/false all lowercase."

sleep 1

(cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=97 -y localhost)&
./2LegMain.py $1 $2 >/dev/null
