#!/bin/bash

(cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=97 -y localhost) && ./2LegMain.py
