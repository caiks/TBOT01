# TBOT01 - TurtleBot3 controller

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we develop a TurtleBot3 controller that has at its core an *alignment induced model*. 

The *model* is trained using the *inducers* and *conditioners* implemented in the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC). The AlignmentRepaC repository is a fast C++ implementation of some of the *practicable inducers* described in the paper *The Theory and Practice of Induction by Alignment* at https://greenlake.co.uk/. The *models* are *induced* in the main executable, which can run without the need to install TurtleBot3.

The *history* (training data) is collected by running various random/demo controllers in the [turtlebot3 house](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#3-turtlebot3-house) simulated in the [Gazebo](http://gazebosim.org/) virtual environment.

## Download, build and run main executable

To run the non-ROS main executable it is only necessary to install the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC) and its underlying repositories. The `AlignmentRepaC` module requires [modern C++](https://en.cppreference.com/w/) version 17 or later to be installed.

For example, in Ubuntu bionic (18.04),
```
sudo apt-get update -y && sudo apt install -y git g++ cmake

```
Then download the zip files or use git to get the TBOT01 repository and the underlying rapidjson, AlignmentC and AlignmentRepaC repositories -
```
cd
git clone https://github.com/Tencent/rapidjson.git
git clone https://github.com/caiks/AlignmentC.git
git clone https://github.com/caiks/AlignmentRepaC.git
git clone https://github.com/caiks/TBOT01.git

```
Then download the [TBOT01 workspace repository](https://github.com/caiks/TBOT01_ws) -
```
git clone https://github.com/caiks/TBOT01_ws.git

```
Then build -
```
cp TBOT01/CMakeLists_noros.txt TBOT01/CMakeLists.txt
mkdir TBOT01_build
cd TBOT01_build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../TBOT01
make

```

```
cd ../TBOT01_ws
ln -s ../TBOT01_build/main main

./main stats 202001222010_2.TBOT01

./main stats 202001271320_room1.TBOT01

./main bitmap 202001222010_2.TBOT01

./main bitmap 202001222010_2.TBOT01 3

./main bitmap 202001271320_room1.TBOT01

./main bitmap_average 202001222010_2.TBOT01

./main bitmap_average 202001271320_room1.TBOT01 20

./main bitmap_average 202001271320_room4.TBOT01 20

./main induce model001 4 >model001.log

./main bitmap_model model001 

./main induce model002 4 >model002.log

./main bitmap_model model002 

./main induce model003 4 >model003.log

./main bitmap_model model003 

./main induce model004 4 >model004.log

./main bitmap_model model004 

./main induce model005 4 >model005.log

./main bitmap_model model005 

./main condition model006 4 motor >model006_motor.log

./main bitmap_model model006_motor 

./main test model006_motor motor 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 890
frund(*dr->fud)->size(): 97
frvars(*dr->fud)->size(): 1113
model: model006_motor
label: motor
effective size: 6054
matches: 5849

./main condition model006 4 location >model006_location.log

./main bitmap_model model006_location

./main test model006_location location 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 890
frund(*dr->fud)->size(): 97
frvars(*dr->fud)->size(): 1113
model: model006_location
label: location
effective size: 6054
matches: 5425

./main condition model006 4 position >model006_position.log

./main bitmap_model model006_position

./main test model006_position position 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 890
frund(*dr->fud)->size(): 103
frvars(*dr->fud)->size(): 1119
model: model006_position
label: position
effective size: 6054
matches: 5590

./main condition model007 4 motor >model007_motor.log

./main bitmap_model model007_motor 

./main test model007_motor motor 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 128
frund(*dr->fud)->size(): 331
frvars(*dr->fud)->size(): 2792
model: model007_motor
label: motor
effective size: 6054
matches: 5669

./main condition model007 4 location >model007_location.log

./main bitmap_model model007_location

./main test model007_location location 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 128
frund(*dr->fud)->size(): 295
frvars(*dr->fud)->size(): 2060
model: model007_location
label: location
effective size: 6054
matches: 4985

./main condition model007 4 position >model007_position.log

./main bitmap_model model007_position

./main test model007_position position 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 128
frund(*dr->fud)->size(): 330
frvars(*dr->fud)->size(): 2512
model: model007_position
label: position
effective size: 6054
matches: 5209

./main induce model008 4 >model008.log

```
## Download, build and run controller executable

To run the controller it is necessary to install [ROS2](https://index.ros.org/doc/ros2/), [Gazebo](http://gazebosim.org/tutorials?cat=install) and [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_simulation/#simulation) on a machine with a GPU.

### AWS EC2 instance

For example on [AWS EC2](https://aws.amazon.com/ec2/) start a `g2.2xlarge` Ubuntu bionic (18.04) instance.

Set the enivronment variables required by `ssh`,
```
h=ubuntu@ec2-18-203-247-60.eu-west-1.compute.amazonaws.com
k=kp01.pem

```
Then connect with X11 forwarding enabled,
```
ssh -X -i $k $h

```
Then install as follows,
```
sudo apt-get update -y && sudo apt install -y g++ xorg gedit

sudo reboot

```

```
ssh -X -i $k $h

```

```
sudo apt install -y ubuntu-drivers-common && sudo ubuntu-drivers autoinstall

sudo reboot

ssh -X -i $k $h

nvidia-smi -q | head

```
The GPU drivers should be working at this point.

For example, test the GPU with GLX gears, 
```
sudo apt-get install -y mesa-utils

glxgears

```
There should be a set of cogs smoothly rotating.

The EC2 instance is ready to proceed with the remainder of the [installation](#Installation).

### Windows 10 WSL2 instance

Another example is an instance of a Windows 10 machine with a GPU, running [WSL2](https://docs.microsoft.com/en-us/windows/wsl/wsl2-index).

Install [Ubuntu 18.04](https://www.microsoft.com/en-gb/p/ubuntu-1804-lts/9n9tngvndl3q?rtc=1&activetab=pivot:overviewtab).

Check the virtual machine is running at WSL2 in admin powershell
```
wsl -l -v

```

Launch Ubuntu, create the user and then prepare it for Xwindows. Add the following line to `~/.bashrc`,
```
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

```
Install Xwindows,
```
source ~/.bashrc

sudo apt update

sudo apt install -y g++ xorg gedit

```
Install [VcVsrv](https://sourceforge.net/projects/vcxsrv/files/latest/download).

Run Xlaunch: reset `Native opengl` and set `Disable access control`.

Test with `xeyes`. 

Note that Windows Firewall may be blocking, see   https://github.com/microsoft/WSL/issues/4171#issuecomment-559961027
 
### Installation

Now install Gazebo9,
```
sudo apt-get install -y gazebo9 libgazebo9-dev

gazebo -v

```
Install ROS2 Eloquent,
```
sudo apt install -y curl gnupg2 lsb-release

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update

sudo apt install -y ros-eloquent-desktop ros-eloquent-gazebo-* ros-eloquent-cartographer ros-eloquent-cartographer-ros

echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
Test by running these nodes in separate shells,
```
ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_py listener

```
Install TurtleBot3,

```
sudo apt install -y python3-argcomplete python3-colcon-common-extensions google-mock libceres-dev liblua5.3-dev libboost-dev libboost-iostreams-dev libprotobuf-dev protobuf-compiler libcairo2-dev libpcl-dev python3-sphinx python3-vcstool

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos
colcon build --symlink-install

echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc

export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}

```
Test with
```
ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```

Now download and build the TBOT01 repository and the underlying rapidjson, AlignmentC and AlignmentRepaC repositories -
```
cd ~/turtlebot3_ws/src

git clone https://github.com/Tencent/rapidjson.git
git clone https://github.com/caiks/AlignmentC.git
git clone https://github.com/caiks/AlignmentRepaC.git
git clone https://github.com/caiks/TBOT01.git
git clone https://github.com/caiks/TBOT01_ws.git

mkdir AlignmentC_build AlignmentRepaC_build
cd ~/turtlebot3_ws/src/AlignmentRepaC_build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../AlignmentRepaC
make AlignmentC AlignmentRepaC

cd ~/turtlebot3_ws/src/TBOT01
cp CMakeLists_ros.txt CMakeLists.txt

cd ~/turtlebot3_ws
colcon build --packages-select TBOT01

source ~/.bashrc
```

The simiulation can be started in paused mode,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env001.model -s libgazebo_ros_init.so

```
In a separate shell,
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller TBOT01.bin 250 2000 4000

```

