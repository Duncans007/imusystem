# imusystem

Main File: variableSensorMain.py

User Config: userinput.py

Run File: run.sh

----------------------------------------------------------------------------

Putting on the System:

1. There are 8x k-ceptors (sensors), 8x cables, 1x hub, 1x Raspberry Pi, 1x battery.
    All k-ceptors can be toggled on/off in the userinput.py configuration file.
    !!! Be aware that turning certain sensors off (e.g. shank) may cause some detections to fail.
2. Hub, Pi, Battery attach to back brace with velcro.
3. Sensors each have designated locations. All information is on label. 
    
    Left leg (hub slot 3) has red labels.
    
    Right leg (hub slot 1) has silver labels.
    
    Back sensors (hub slot 2) have yellow labels. 
    Lower back sensor attaches to the velcro at the bottom of the back brace.
    Upper back sensor mus be enabled by toggling sensor8 to True, and attaches to velcro at the top of the back.
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

----------------------------------------------------------------------------

First Time Raspberry Pi Setup:

Setting up internet connection
1. Plug in microcontroller to power source using included cable.
2. Plug in HDMI display and keyboard. Only necessary for internet set-up.
3. Run command "sudo raspi-config".
4. Select option "2".
5. Select option "N2".
6. Enter SSID of network - note that capitals and spaces matter.
7. Enter password for network.
8. Exit menus.

Setting up internet connection WITHOUT MONITOR/KEYBOARD:
1. Power raspi down and remove microSD card. Insert into computer.
2. Create file wep-supplicant.conf. See link for contents of file: https://www.raspberrypi-spy.co.uk/2017/04/manually-setting-up-pi-wifi-using-wpa_supplicant-conf/
3. Reinsert microSD card and turn raspi on.

Connecting through a client computer
1. Run command "ifconfig wlan0" on raspi.
    
    note: if setting up without a monitor/keyboard, you WILL need to find a way to get the pi's IP address (nmap, wireshark, router access)
    
    If there is no way of getting the pi's IP, I would recommend a mobile hot-spot (like a phone) that can configure SSID/password to whatever the pi is looking for PLUS show the device's IP address.
    
2. Note the first number on the second line - "inet *.*.*.*"
3. Open terminal on client computer.
4. Run "ssh pi@*.*.*.*" with the IP adress from step 2.
5. Enter password chordata (text does not display as you type).

note: make sure you are connected to the same network.

----------------------------------------------------------------------------

Running the Notochord:

Notochord Executable Location:

/home/pi/notochord/bin/notochord

.

Usage:
./notochord [options] [ip*] [port**]
*For running the algorithm, set to localhost
**Port usually unnecessary

Useful Flags:
-h help, displays more written out version of this short section

-y starts program without waiting for user confirmation

-r gives raw values as output (also --raw)

-x for calibration. Only have one K-Ceptor plugged in (also --calibrate)

-c [path] path to config file. Defaults to home/pi/notochord/Chordata.xsd

-v [0-2] more verbose output, default 0

-l [output] where to redirect log messages

-e [output] where to redirect errors

-t [output] where to redirect data (mostly quaternions)

-s [send_rate] data send rate


--no_bundles unbundles code, do not use with algorithm.
--odr=[frequency] changes sample rate, default 50Hz, max 100Hz
--scan creates armature by scanning setup instead of using configuration file.

.

Updating the Notochord:

Navigate to the Notochord directory

git checkout develop

git pull

scons -j3 debug=1

----------------------------------------------------------------------------

Using the Algorithm

Installing the Algorithm:
run "git pull https://github.com/duncan006/imusystem.git"

.

Updating the Algorithm:

navigate to /imusystem/

git stash

git pull

sudo chmod 777 *.py


.


Running the Algorithm:

1. Use the included run.sh file to start both the Notochord and algorithm at the same time.

    Navigate to the imusystem directory and run the file using:
    ./run.sh


.


Configuring the Algorithm:

1. Open userinput.py using
    nano imusystem/userinput.py
    
    
Subject Parameters:

    mass (kilograms)
    
    height (meters)
    
    
Torque Controllers:

    torqueCutoff (maximum torque)
    
    alpha (yu&su controller proportionality constant)
    
    NMKG (PID controller proportionality constant)
    
    
Serial Communication:

    baud (typically 115200 or 256000, match to system. NUC simulink expects 115200)
    
    port (port that the given device is attached to)
    
        run "ls /dev/tty*" with and without device attached. Note the entry that changes.
        
        ="/dev/ttyS0" (GPIO pins)
        
        ="/dev/ttyUSB0" (generic USB output, ex. a serial cable)
        
        ="/dev/ttyACM0" (USB Microcontroller, ex. arduino)
        

----------------------------------------------------------------------------

LIMITATIONS OF THE SYSTEM:

Currently, this system is derived from the Chordata beta motion capture system. Usually, this data is directly transmitted to a client computer where a custom Blender addon does the calibration.
Running the data through the Blender processing greatly improves the accuracy of the angle measurements from the device. However, it comes at the cost of halving the send rate. (50Hz to ~20Hz)
To take advantage of the higher send rate, I had to write my own algorithm to manually convert the raw values to angles. This algorithm is fairly accurate, but has some limitation to get that accuracy without writing a full sensor fusion algorithm.
The Chordata roadmap estimates that its functions will be reprogrammed to be used more easily natively before the end of 2020

----------------------------------------------------------------------------

ADDING ALGORITHMS

Two files with the TEMPLATE_ prefix are included to add detection algorithms.
To use, make a copy of the file, change the title, and follow the included instructions.
simpleIO can be used for detection algorithms that only use information from a single time frame.
continuous can be used for detection algorithsm that use information over a range of time.
When finished writing the function/object, follow the instructions at the bottom of the file to add appropriate calls in variableSensorMain.py
