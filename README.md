# VisNav
Visual Navigation for ArduPilot

Preqs:
	-Mavproxy
	-dronekit

Setup:
1. Navigate to your home directory
cd 

2. Download my fork of ardupilot
git clone https://github.com/djnugent/ardupilot

3. Download visnav
git clone https://github.com/djnugent/visnav

4. Download the most recent version of pymavlink 
git clone https://github.com/mavlink/mavlink

5. Uninstall any previous versions of pymavlink
sudo pip uninstall pymavlink

6. Navigate into pymavlink directory
cd /mavlink/pymavlink

7. Install pymavlink
sudo python install setup.py

8. Add sim_vehicle.sh to your path
sudo nano ~/.bashrc
ADD THE FOLLOWING TO THE END OF THE FILE:
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
Use ctrl-x to exit and save the file

9. Navigate to ardupilot/ArduCopter
cd /ardupilot/ArduCopter

10. Intialize SITL
sim_vehicle.sh -w

11. Disable prearm checks
param set ARMING_CHECK 0

12. Kill SITL
press: ctrl-c



Run PrecisionLand:
1. Navigate to ardupilot/ArduCopter
cd /ardupilot/ArduCopter

2. Start SITL
sim_vehicle.sh --console --map

3. Arm vehicle
arm throttle

4. Switch to guided mode
mode guided

5. Takeoff
takeoff 20

6. Start PrecsionLand
api start /home/<your_username>/visnav/PrecisionLand.py

7. Kill SITL
Press: ctrl-c