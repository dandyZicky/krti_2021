#!/bin/bash

# open a new terminal window and run roslaunch krti_2021 vtol_iqsim.launch
gnome-terminal -- roslaunch krti_2021 vtol_iqsim.launch

# wait for a few seconds before opening the next terminal window
sleep 1

# open a new terminal window and run sim_vehicle.py -v ArduCopter -f gazebo-iris --console
gnome-terminal -- sim_vehicle.py -v ArduCopter -f gazebo-iris --console

# wait for a few seconds before opening the next terminal window
sleep 1

# open a new terminal window and run roslaunch iq_sim apm.launch
gnome-terminal -- roslaunch iq_sim apm.launch

