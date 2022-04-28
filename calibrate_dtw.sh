#!/bin/bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT


echo "Running Dynamic Time Warping Calibration..."
echo "Calibration Steps:"
echo "1. Stand still for 10 seconds for initial pose capture."
echo "2. Then, walk in a straight line as far as possible."
echo "3. DO NOT TURN. ONLY STOP ONCE."
echo "-----------------------------------------------"
echo "-----------------------------------------------"
echo "usable arguments include \"save=\" and \"load=\" with filenames."
echo "save argument requires no extension"
echo "load argument must be .txt or .csv output from imu system."
echo "-----------------------------------------------"
echo "-----------------------------------------------"
echo "Test will auto-terminate after 40 seconds total."
echo "this includes startup and sensor calibration time."
echo "Total time will begin in 10 seconds."

sleep 10

echo "-----------------------------------------------"
echo "-----------------------------------------------"
echo "Test starting, please stand still."
sleep 1
echo "3"
sleep 1
echo "2"
sleep 1
echo "1"
sleep 1
echo "running..."

(cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=97 -y localhost)&

sleep 6

./variableSensorMain.py& #$1 $2 $3 #>/dev/null

sleep 10

echo "-----------------------------------------------"
echo "-----------------------------------------------"
echo "Initial Pose Capture Complete."
echo "Begin Walking."

sleep 24

PID=$(jobs -p)

kill $PID

echo "-----------------------------------------------"
echo "-----------------------------------------------"
echo "Data Collection Complete"

sleep 1

echo "Processing..."

./dtw_calibration.py

sleep 2

echo "-----------------------------------------------"
echo "-----------------------------------------------"
echo "Calibration Complete."