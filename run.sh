#!/bin/bash

./2LegMain.py -$1 -$2 && (cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=97 -y localhost)
