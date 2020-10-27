#!/bin/bash

echo "Both arguments are True/False, capital first letter. First is NUC. Second is Teensy."

sleep 1

./2LegMain.py -$1 -$2 && (cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=97 -y localhost)
