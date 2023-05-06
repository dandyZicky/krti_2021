pkill -f "roslaunch krti_2021 vtol_iqsim.launch"
pkill -f "sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
pkill -f "roslaunch iq_sim apm.launch"
