# Installation Instructions

## Installing ROS

Install ROS Noetic from http://wiki.ros.org/noetic/Installation/Ubuntu

## Installing Ardupilot and MAVProxy Ubuntu 20.04

### Video Tutorial at https://youtu.be/1FpJvUVPxL0

### Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

### Install dependencies:
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

reload profile
```
. ~/.profile
```

### Checkout Latest Copter Build
```
git checkout Copter-4.2
git submodule update --init --recursive
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
## Installing Gazebo and ArduPilot Plugin

### Video Tutorial at https://youtu.be/m7hPyJJmWmU

### Overview 

Robot simulation is an essential tool in every roboticist's toolbox. A well-designed simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. At your fingertips is a robust physics engine, high-quality graphics, and convenient programmatic and graphical interfaces. Best of all, Gazebo is free with a vibrant community.

for more infromation on gazebo checkout http://gazebosim.org/

### Install Gazebo 

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```

Install Gazebo:

```
sudo apt-get install gazebo11 libgazebo11-dev
```

for more detailed instructions for installing gazebo checkout http://gazebosim.org/tutorials?tut=install_ubuntu


### Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```
## Setting up the workspace
```
cd
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Blackhawk2624/Blackhawk_PS.git
cd ..
catkin_make
```

## Environment Setup
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Note

1. Extract the content of default_params.zip and paste gazebo-drone1.parm, gazebo-drone2.parm ... gazebo-drone12.parm in /home/username/ardupilot/Tools/autotest/default_params directory
2. Replace /home/username/ardupilot/Tools/autotest/pysim/vehicleinfo.py file with vehicleinfo.py(present in this repository) file
3. Extract the content of model_drones.zip and paste drone1, drone2 ... drone12 in /home/username/.gazebo/models directory

## Running the world

In Terminal 1
```
roslaunch iq_sim blackhawk.launch 
```
In Terminal 2
```
roscd iq_sim
./blackhawk.sh
OR
bash blackhawk.sh
```
## World
![iq](world.jpg)

## References

1. https://github.com/Intelligent-Quads/iq_tutorials
2. https://youtu.be/r15Tc6e2K7Y
3. https://youtu.be/UWsya46ZG4M
4. https://youtu.be/kcCL0w4NbIc

