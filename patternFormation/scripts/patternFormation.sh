#!/bin/bash

python_directory="/home/mohit/workspace/drone_ws/src/drone_swarming/patternFormation/assist_scripts"  # Replace with the actual path to the directory where your Python files are located

gnome-terminal -- bash -c "cd $python_directory; \
gnome-terminal --tab --title='drone1' -- bash -c 'python drone1.py; exec bash'; \
gnome-terminal --tab --title='drone2' -- bash -c 'python drone2.py; exec bash'; \
gnome-terminal --tab --title='drone3' -- bash -c 'python drone3.py; exec bash'; \
gnome-terminal --tab --title='drone4' -- bash -c 'python drone4.py; exec bash'; \
gnome-terminal --tab --title='drone5' -- bash -c 'python drone5.py; exec bash'; \
gnome-terminal --tab --title='drone6' -- bash -c 'python drone6.py; exec bash'; \
gnome-terminal --tab --title='drone7' -- bash -c 'python drone7.py; exec bash'; \
gnome-terminal --tab --title='drone8' -- bash -c 'python drone8.py; exec bash'; \
gnome-terminal --tab --title='drone9' -- bash -c 'python drone9.py; exec bash'; \
gnome-terminal --tab --title='drone10' -- bash -c 'python drone10.py; exec bash'; \
gnome-terminal --tab --title='drone11' -- bash -c 'python drone11.py; exec bash'; \
gnome-terminal --tab --title='drone12' -- bash -c 'python drone12.py; exec bash';"
