#!/bin/bash

gnome-terminal -- bash -c "
gnome-terminal --tab --title='Vehicle1' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0; exec bash'; \
gnome-terminal --tab --title='Vehicle2' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I1; exec bash'; \
gnome-terminal --tab --title='Vehicle3' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I2; exec bash'; \
gnome-terminal --tab --title='Vehicle4' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I3; exec bash'; \
gnome-terminal --tab --title='Vehicle5' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I4; exec bash'; \
gnome-terminal --tab --title='Vehicle6' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I5; exec bash'; \
gnome-terminal --tab --title='Vehicle7' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I6; exec bash'; \
gnome-terminal --tab --title='Vehicle8' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I7; exec bash'; \
gnome-terminal --tab --title='Vehicle9' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I8; exec bash'; \
gnome-terminal --tab --title='Vehicle10' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I9; exec bash'; \
gnome-terminal --tab --title='Vehicle11' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I10; exec bash'; \
gnome-terminal --tab --title='Vehicle12' -- bash -c 'sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I11; exec bash';"
