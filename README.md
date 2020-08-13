# imusystem

----------------------------------------------------------------------------

Putting on the System:

1. There are 7x k-ceptors (sensors), 7x cables, 1x hub, 1x Raspberry Pi, 1x battery.
2. Hub, Pi, Battery attach to back brace with velcro.
3. Sensors each have designated locations. All information is on label. 
    Left leg (hub slot 3) has red labels.
    Right leg (hub slot 1) has silver labels.
    Back sensors (hub slot 2) have yellow lables. There is only one back sensor in use currently. It attaches to the velcro at the bottom of the back brace.
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

Updating the Notochord:
Navigate to the Notochord directory

git checkout develop
git pull
scons -j3 debug=1

----------------------------------------------------------------------------

Using the Algorithm

Installing the Algorithm:
run "git pull https://github.com/duncan006/imusystem.git"


Updating the Algorithm:

navigate to /imusystem/
git stash
git pull
sudo chmod 777 *.py


Running the Algorithm:

1. Follow steps above to run the Notochord, using "./notochord/bin/notochord --raw --scan --odr=97 --no_bundles -y localhost"
2. Run 2LegMain.py in ./imusystem
note: 1LegMain.py never had serial communication implemented, as it was never necessary. Will update.

note: the algorithm dumps to the same file every time (algDump.txt). If you want to save your results on the pi, run "sudo mv algDump.txt desiredName.txt"
        the algDump.txt output is formatted to be pasted directly into any spreadsheet program with proper spacing and no extra work.
        to grab results from the pi on the computer, use "scp pi@IP:imusystem/algDump.txt desiredLocation.txt"


Running the Algorithm (Cheat Sheet):

Fill in IP
ssh pi@IP ./notochord/bin/notochord --raw --scan --odr=97 --no_bundles -y localhost
ssh pi@IP ./imusystem/2LegMain.py

----------------------------------------------------------------------------

LIMITATIONS OF THE SYSTEM:

Currently, this system is derived from the Chordata beta motion capture system. Usually, this data is directly transmitted to a client computer where a custom Blender addon does the calibration.
Running the data through the Blender processing greatly improves the accuracy of the angle measurements from the device. However, it comes at the cost of halving the send rate. (50Hz to ~20Hz)
To take advantage of the higher send rate, I had to write my own algorithm to manually convert the raw values to angles. This algorithm is fairly accurate, but has some limitation to get that accuracy.
The algorithm cancels drift by using gait stages. When the subject is standing, the angles get re-zeroed. Thus, the angles are only accurate during walking and up until the end of a slip.
It is in the process of being updated to use additional sensor readings to improve accuracy, and allow the angles to be accurate for kneeling.
As it is now, accurate kneeling angles can only be obtained if kneeling then immediately standing up. (not giving the algorithm time to detect standing still)

The slip algorithm itself works well but has false positives sometimes, particularly if the toe is pointed directly at the floor and the accelerometer reading is distorted due to gravity.
Like I said about the angles, it is most accurate during normal gait, with future updates coming to get it more accurate after collecting data from actual slip with harness. (all previous tests have been faux slip)

----------------------------------------------------------------------------

Troubleshooting:
