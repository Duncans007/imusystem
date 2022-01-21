# imusystem

imusystem is a set of Python functions and classes to:
1) Process the output OSC stream from the Chordata Notochord program, which gives Gyroscope and Accelerometer measurements.
2) Analyze the system kinematics for absolue and relative angle measurements, perturbation detection, and assistive device control.

The program is built to collect data from 8 maximum Inertial Measurement Units (IMUs) on the foot, shank, thigh, and upper&lower back.

## Simple Info

### File Overview

Main File: variableSensorMain.py

User Config: userinput.py

Run File: run.sh

### Dependencies

- pythonosc
- numpy
- [Chordata Notochord](https://gitlab.com/chordata/notochord) is not a Python dependency. The "notochord" folder must be placed in the same directory as the "imusystem" folder.

## Putting on the System:

1. There are 8x k-ceptors (sensors), 8x cables, 1x hub, 1x Raspberry Pi, 1x battery.
    All k-ceptors can be toggled on/off in the userinput.py configuration file.
    !!! Be aware that turning certain sensors off (e.g. shank) may cause some detections to fail.
2. Hub, Pi, Battery attach to back brace with velcro.
3. Sensors each have designated locations. All information is on label. 
    
    Left leg (hub slot 3) has red labels.
    
    Right leg (hub slot 1) has silver labels.
    
    Back sensors (hub slot 2) have yellow labels. 
    
    Lower back sensor attaches to the velcro at the bottom of the back brace.
    
    Upper back sensor must be enabled by toggling sensor8 to True, and attaches to velcro at the top of the back.
    
4. Sensors also have orientation lablels for X, Y and Z axes. Line these up so that on every sensor the...
    
    Y arrow points UP
    
    X arrow points FORWARD
    
    Z arrow points out of the sagittal plane to the RIGHT
    
    Try to keep sensors, especially the heel sensor, as aligned as possible to this framework. Make sure that the heel sensor is parallel to the ground.
    
    Sensors are oriented to go on the outside of the leg. If you would prefer to put them on the inside, switch the left and right legs and still match the coordinate axes.
    
5. Once all sensors are attached to the subject and oriented correctly, connect the cables.
    
    Each sensor has an IN arrow and OUT arrow.
    
    Cables coming FROM the HUB or sensor before connect to the IN arrow.
    
    Cables going TO the NEXT SENSOR in the string connect to the OUT arrow.
    
6. Put on the back brace and make final connections.
    
    Before putting on the back brace, make sure that the lower back sensor is plugged in to hub slot 2, and the two longest cables are plugged into hub slots 1 and 3.
    
    Make sure that the Raspberry Pi is getting power and the blue LED in the lower back sensor is on.
    
    The easiest way to assemble the system without a helper is to leave these two cables hanging until the back brace is on. Then, they can easily be plugged into the IN arrow on the thigh sensors.

## Usage

### Run

The imusystem can be run by using the included bash file to automatically start both the Notochord and Python receiver at the same time.

```
./run.sh
```

Program execution can be stopped at any time by mashing ctrl+c until it gives up (to be improved).

Program continuously dumps data to ```algDump.txt``` which includes tab separators so it can be pasted into any cell editor (Excel, Google Sheets, etc.). Output can be converted to .csv in future for easier reading.

**NOTE:** **_```algDump.txt``` is overwritten every time the program starts. To ensure data is not lost, copy the file to another location (preferably the connected host computer) before running another test._**

### Configuration

Most user configuration options are contained in ```userinput.py```.

#### Subject Parameters

Edit to match the subject wearing the system. Many assumed lengths and ratios are estimated from these values. Constants taken from Winter's Biomechanics Chapter 4.
```
mass (kilograms)

height (meters)
```

#### Sensor Config

Sensors have a small amount of configurability through Python. Individual sensors can be switched on/off and calibration times at the beginning of each test can be changed. Subject must stand still for combined duration of calibration times and Notochord startup time at the beginning of each test.

Approximate processing frequency can also be increased or decreased if necessary. This changes the regular interval at which the current data received is processed and saved, but is not able to increase the rate at which data is received from the Notochord.

```
processing_frequency (Hz)

toggle_[SENSOR] (True/False)

sensorCalibTime (seconds to eliminate gyroscope noise)
angleCalibTime (seconds to zero angle measurements after sensorCalibTime is complete)
```
    
#### Torque Controllers:

The IMU system is sometimes used to control variable torque knee devices over a serial output. These can be configured based on the type of controller selected. See ```userinput.py``` for more information.
```
controller_type (pid, yusu, ramp, trkov)
# "pid" - DS developed, uses knee angle and thigh ang. vel.
# "yusu" - Based on paper by Yu&Su. Uses thigh kinematics plus lower back.
# "ramp" - Constant ramping torque.
# "trkov" - Developed by MT, adapted by DS, uses full lower body kinematics plus lower back.
```
    
#### Serial Communication:

IMU system can send and receive over serial to communicate with a variety of devices:

1) Send to a MATLAB receiver over a RS232 cable connected with the Raspberry Pi GPIO.
2) Send to or receive from an Arduino for device control or sensor readings respectively.

This can be further configured using the options given below:

```
baud (typically 115200 or 256000, match to system. NUC simulink expects 115200)
port (port that the given device is attached to)
# run "ls /dev/tty*" with and without device attached. Note the entry that changes.
# ="/dev/ttyS0" (GPIO pins)
# ="/dev/ttyUSB0" (generic USB output, ex. a serial cable)
# ="/dev/ttyACM0" (USB Microcontroller, ex. arduino)
```
        


## LIMITATIONS OF THE SYSTEM:

Currently, this system is derived from the Chordata beta motion capture system. Usually, this data is directly transmitted to a client computer where a custom Blender addon does the calibration.
Running the data through the Blender processing greatly improves the accuracy of the angle measurements from the device. However, it comes at the cost of halving the send rate. (50Hz to ~20Hz)
To take advantage of the higher send rate, I had to write my own algorithm to manually convert the raw values to angles. This algorithm is fairly accurate, but has some limitation to get that accuracy without writing a full sensor fusion algorithm.
The Chordata roadmap estimates that its functions will be reprogrammed to be used more easily natively before the end of 2020


## ADDING ALGORITHMS

Two files with the TEMPLATE_ prefix are included to add detection algorithms.
To use, make a copy of the file, change the title, and follow the included instructions.
simpleIO can be used for detection algorithms that only use information from a single time frame.
continuous can be used for detection algorithsm that use information over a range of time.
When finished writing the function/object, follow the instructions at the bottom of the file to add appropriate calls in variableSensorMain.py
