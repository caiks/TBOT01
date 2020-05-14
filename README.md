# TBOT01 - TurtleBot3 controller

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we develop a TurtleBot3 controller that has at its core an *alignment induced model*. 

The *model* is trained using the *inducers* and *conditioners* implemented in the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC). The AlignmentRepaC repository is a fast C++ implementation of some of the *practicable inducers* described in the paper *The Theory and Practice of Induction by Alignment* at https://greenlake.co.uk/. The *models* are *induced* in the main executable, which can run without the need to install TurtleBot3.

The *history* (training data) is collected by running various random/demo controllers in the [turtlebot3 house](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#3-turtlebot3-house) simulated in the [Gazebo](http://gazebosim.org/) virtual environment.

## Sections

[Download, build and run main executable](#main)

[Download, build and run controller executable](#controller)

[Discussion](#Discussion)

<a name="main"></a>

## Download, build and run main executable

To run the non-ROS main executable it is only necessary to install the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC) and its underlying repositories. The `AlignmentRepaC` module requires [modern C++](https://en.cppreference.com/w/) version 17 or later to be installed.

For example, in Ubuntu bionic (18.04),
```
sudo apt-get update -y && sudo apt install -y git g++ cmake

```
Then download the zip files or use git to get the `TBOT01` repository and the underlying `rapidjson`, `AlignmentC` and `AlignmentRepaC` repositories -
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

./main stats data001

./main stats data002_room1

./main bitmap data001

./main bitmap data001 3

./main bitmap data002_room1

./main bitmap_average data001

./main bitmap_average data002_room1 20

./main bitmap_average data002_room4 20

./main history

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
<a name = "controller"></a>

## Download, build and run controller executable

To run the controller it is necessary to install [ROS2](https://index.ros.org/doc/ros2/), [Gazebo](http://gazebosim.org/tutorials?cat=install) and [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_simulation/#simulation) on a machine with a GPU.

<a name = "AWS"></a>

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

<a name = "Windows"></a>

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
 
<a name = "Installation"></a>

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
To test launch one of these worlds,
```
ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```
Then in a separate shell,
```
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}

ros2 run turtlebot3_teleop teleop_keyboard

```
Check that you can steer the turtlebot using the w/x and a/d keys.

Now download and build the `TBOT01` repository and the underlying `rapidjson`, `AlignmentC` and `AlignmentRepaC` repositories -
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

The simulation can be started in paused mode,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env001.model -s libgazebo_ros_init.so

```
In a separate shell,
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data.bin 250

```
Press play in `gazebo` and the turtlebot3 will start moving.

To run the non-ros `main` executable, create a link,

```
cd ~/turtlebot3_ws/src/TBOT01_ws
ln -s ~/TBOT01_build/main main

```

<a name = "Discussion"></a>

## Discussion

Now let us investigate various turtlebot3 controllers. 

In the following, the rooms are numbered 1 to 6 in the [turtlebot3 house](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#3-turtlebot3-house) from top-left to bottom-right.

Let us start the turtlebot from room 4. We set the turtlebot pose in  [env001.world](https://github.com/caiks/TBOT01_ws/blob/master/env001.model),
```xml
<include>
  <pose>-2.0 1.5 0.01 0.0 0.0 0.0</pose>
  <uri>model://turtlebot3_burger</uri>
</include>
```
Run the simulation,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

cd ~/turtlebot3_ws/src/TBOT01_ws

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env001.model -s libgazebo_ros_init.so

```
Now run the `turtlebot3_drive` controller in the [turtlebot3_gazebo package](http://wiki.ros.org/turtlebot3_gazebo) (note that the documentation refers to ROS1 but we are using ROS2). 

In a separate shell, run
```
ros2 run turtlebot3_gazebo turtlebot3_drive

```
The turtlebot moves around room 4 before moving to the corridor between room 4 and room 1. It passes through the front door and will carry on indefinitely outside. 

Note that in this simulation and the others below, small variations in the timing of the messages sent from the controller can cause quite large differences in the turtlebot's path after a few minutes, so these experiments are not exactly reproducible.

The [Turtlebot3Drive node](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/ros2/turtlebot3_gazebo/include/turtlebot3_gazebo/turtlebot3_drive.hpp) does simple collision avoidance. It subscribes to the lidar `scan` and odometry `odom` topics, and publishes to the motor `cmd_vel` topic. It runs a timer every 10 ms which calls a callback where the direction is decided. The `scan` data is first checked at 0 deg. If there is an obstacle ahead it turns right, otherwise the `scan` data is checked at 330 deg. If there is an obstacle to the left it turns right, otherwise the `scan` data is checked at 30 deg. If there is an obstacle to the right it turns left. If there are no obstacles it drives straight ahead. Once the direction is decided a `geometry_msgs::msg::Twist` is published to `cmd_vel` either with (a) a linear motion at 0.7 m/s, or (b) a rotation clockwise or anti-clockwise at 1.5 rad/s. While rotating the controller waits until the orientation has changed by 30 deg before the direction is decided again.

In general the `turtlebot3_drive` controller does not collide very often with the walls, but can sometimes collide with table legs. There is a preference for right turns over left, so overall motion is usually clockwise.

For our investigations we copied the `turtlebot3_drive` controller to the `TBOT01` [controller](https://github.com/caiks/TBOT01/blob/master/controller.h) and added extra functionality.

Restart the simulation and in a separate shell run the controller,
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data.bin 250

```
If there are only two arguments the `TBOT01` controller behaves like `turtlebot3_drive` but will in addition regularly write records defined by the `Record` structure in [dev.h](https://github.com/caiks/TBOT01/blob/master/dev.h),
```cpp
struct Record
{
...
	std::size_t id;
	double ts;
	double sensor_pose[7];
	double sensor_scan[360];
	double action_linear;
	double action_angular;
};
```
`sensor_pose` records the odometry, `sensor_scan` records the lidar, and `action_linear` and `action_angular` records the motor action. In this case the records are written to `data.bin` every 250 milliseconds. 

Run `TBOT01` for around 1 minute. Stop it by killing the controller to close the records file, and then by pressing pause in Gazebo to pause the simulation. If the turtlebot did not collide with the letterbox it should have completely left the grounds of the house. Let us examine the records generated,
```
./main stats data

```
The `stats` procedure reads the `data.bin` records file and writes some statistics. This is a typical output,
```
rr->size(): 253
rr->front().id: 0
rr->front().ts: 19.0016
rr->front(): (0,19.0016,(-1.99886,1.49979,0.00845354,0.000297038,0.000926921,-0.217572,0.976044),(3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,2.01354,1.96314,1.98215,1.96017,1.96184,1.96562,1.93584,1.93213,1.93106,1.9239,1.92403,1.92239,1.91795,1.88348,1.90323,1.90979,1.91851,1.90812,1.89889,1.9007,1.89326,1.89709,1.91238,1.90077,1.91721,1.92671,1.91716,1.93235,1.94111,1.93617,1.97552,1.94811,1.94905,1.95598,1.98973,1.98053,1.97897,2.01857,2.01352,2.04063,2.04114,2.04763,2.06706,2.09635,2.12213,2.12722,2.14704,2.18101,2.18564,2.22145,2.24016,2.29842,2.27368,2.31563,2.36267,2.36403,2.40737,2.43636,2.46838,2.5098,2.53441,2.59356,2.65222,2.67643,2.72652,2.77643,2.82835,2.89304,2.93364,3.00006,3.07087,3.13889,3.23378,3.28294,3.39114,3.47619,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,0.528108,0.504098,0.514605,0.523534,3.5,1.25786,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,1.81209,1.85076,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,1.40073,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.10323,3.09248,3.07916,3.08373,3.07512,3.07921,3.05579,3.06363,3.06089,3.06902,3.0359,3.04713,3.06089,3.04551,3.0466,3.0641,3.05288,3.06265,3.0641,3.07791,3.07216,3.08957,3.09638,3.11026,3.12779,3.13265,3.14457,3.16801,3.1955,3.19499,3.22608,3.25486,3.25772,3.29746,3.34152,3.34164,3.35618,3.40821,3.43096,3.41207,3.33215,3.20879,3.11637,3.00932,2.95643,2.87021,2.80742,2.73314,2.68891,2.6187,2.55494,2.49519,2.44635,2.39965,2.37801,2.3376,2.27892,2.24385,2.18804,2.16639,2.12982,2.09373,2.07551,2.0589,2.0005,1.98092,1.95251,1.95438,1.91254,1.88802,1.87863,1.87258,1.84507,1.82271,1.80676,1.80905,1.78896,1.75242,1.76739,1.74944,1.72206,1.71095,1.71031,1.70997,1.69238,1.70132,1.67869,1.66119,1.6465,1.65964,1.64122,1.62384,1.64192,1.62911,1.63626,1.61897,1.63313,1.62052,1.61055,1.60077,1.61647,1.60191,1.62089,1.62285,1.63563,1.61777,1.62076,1.62741,1.62431,1.64093,1.62697,1.62869,1.64159,1.65251,1.64563,1.66283,1.68701,1.69108,1.68016,1.68495,1.71728,1.71172,1.75365,1.7512,1.75205,1.77665,1.79278,1.7878,1.82231,1.85437,1.85655,1.85646,1.88569,1.90832,1.92676,1.96801,1.98336,2.00569,2.0358,2.06591,2.08489,2.12244,2.1499,2.18783,2.22221,2.2514,2.30995,2.34242,2.37791,2.43246,2.48282,2.53853,2.59634,2.65323,2.71388,2.77554,2.84279,2.91219,3.00156,3.08293,3.5,3.5,3.5,3.5),0,-1.5)
rr->back().id: 252
rr->back().ts: 82.0015
rr->back(): (252,82.0015,(9.44059,-8.91209,0.00853602,0.000732516,0.00328479,-0.251896,0.967749),(3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5),0.3,0)
i: 0
sensor_pose[i].front(): -1.99886
sensor_pose[i].back(): 9.44059
i: 1
sensor_pose[i].front(): -8.91209
sensor_pose[i].back(): 1.49979
i: 2
sensor_pose[i].front(): 0.00831223
sensor_pose[i].back(): 0.00874165
i: 3
sensor_pose[i].front(): -0.00221269
sensor_pose[i].back(): 0.00384245
i: 4
sensor_pose[i].front(): -0.010905
sensor_pose[i].back(): 0.0126168
i: 5
sensor_pose[i].front(): -0.614916
sensor_pose[i].back(): 0.141104
i: 6
sensor_pose[i].front(): 0.788588
sensor_pose[i].back(): 0.999857
sensor_scan.front(): 0.296369
sensor_scan.back(): 3.5
action_linear.front(): 0
action_linear.back(): 0.3
action_angular.front(): -1.5
action_angular.back(): 1.5
sensor_scan.size(): 91080
sensor_scan_dist: {(0.5,2137),(1,7416),(1.5,5537),(2,5330),(2.5,4053),(3,3285),(3.5,63322),(4,0)}
sensor_scan_limit: 59750
i: 0
sensor_scan[i]: 0.296369
i: 4475
sensor_scan[i]: 0.59146
i: 8950
sensor_scan[i]: 0.94238
i: 13425
sensor_scan[i]: 1.34899
i: 17900
sensor_scan[i]: 1.74842
i: 22375
sensor_scan[i]: 2.21977
i: 26850
sensor_scan[i]: 2.85404
i: 31325
sensor_scan[i]: 3.49973
action_linear_dist: {0,0.3}
action_angular_dist: {-1.5,0,1.5}
```
Here is an example of one we created earlier,
```
./main stats data001

```
with output
```
rr->size(): 279
rr->front().id: 0
rr->front().ts: 0.252699
...
rr->back().id: 278
rr->back().ts: 69.7564
...
i: 0
sensor_pose[i].front(): -1.97511
sensor_pose[i].back(): 4.35613
i: 1
sensor_pose[i].front(): -11.2565
sensor_pose[i].back(): 1.50003
i: 2
sensor_pose[i].front(): 0.0083258
sensor_pose[i].back(): 0.0107955
i: 3
sensor_pose[i].front(): -0.0210602
sensor_pose[i].back(): 0.00900391
i: 4
sensor_pose[i].front(): -0.0261257
sensor_pose[i].back(): 0.0302976
i: 5
sensor_pose[i].front(): -0.972794
sensor_pose[i].back(): 0.150416
i: 6
sensor_pose[i].front(): 0.231651
sensor_pose[i].back(): 0.999958
sensor_scan.front(): 0.12
sensor_scan.back(): 3.5
action_linear.front(): 0
action_linear.back(): 0.3
action_angular.front(): -1.5
action_angular.back(): 1.5
sensor_scan.size(): 100440
sensor_scan_dist: {(0.5,4391),(1,17001),(1.5,8267),(2,5194),(2.5,2919),(3,2160),(3.5,60508),(4,0)}
sensor_scan_limit: 58317
i: 0
sensor_scan[i]: 0.12
i: 6017
sensor_scan[i]: 0.524778
i: 12034
sensor_scan[i]: 0.644876
i: 18051
sensor_scan[i]: 0.848718
i: 24068
sensor_scan[i]: 1.1434
i: 30085
sensor_scan[i]: 1.53067
i: 36102
sensor_scan[i]: 2.18768
i: 42119
sensor_scan[i]: 3.49977
action_linear_dist: {0,0.3}
action_angular_dist: {-1.5,0,1.5}
```
We can visualise the `scan` data in a bitmap,
```
./main bitmap data

```
Open `data.bmp` in an image viewer. The 360 scan rays are represented horizontally with 0 deg in the centre. The brightness indicates the distance detected, brighter for foreground objects and darker for background. It is black where there are no obstacles within the 3.5 m maximum range of the lidar. The time axis is vertical from bottom to top. Here is the image for `data001`,

![data001](images/data001.jpg?raw=true)

We can see the turtlebot zig-zagging around frequently in the corridor and less often outside the house, until all obstacles are out of range of the lidar sensor.

Generate an average of the records,
```
./main bitmap_average data001

```
and view it in `data001_average.bmp`,

![data001_average](images/data001_average.bmp?raw=true)

Now let us close the door by placing an obstacle there. We will also remove tables and chairs. The new [turtlebot3_house001](https://github.com/caiks/TBOT01_ws/blob/master/gazebo_models/turtlebot3_house001/model.sdf) is included in  [env002.world](https://github.com/caiks/TBOT01_ws/blob/master/env002.model). 

The simulation was restarted,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
and the controller re-run for around 5 minutes,
```
ros2 run TBOT01 controller data002_room4.bin 250

```
Again we can examine the statistics,
```
./main stats data002_room4

```
Because of its tendency to right turns, the turtlebot stays in in room 4. 

Now let us restart the turtlebot in room 1,

```xml
<include>
  <pose>3.575430 4.271000 0.01 0.0 0.0 0.0</pose>
  <uri>model://turtlebot3_burger</uri>
</include>
```
in  [env003.world](https://github.com/caiks/TBOT01_ws/blob/master/env003.model).

The simulation was restarted,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env003.model -s libgazebo_ros_init.so

```
Again the controller is run for around 5 minutes,
```
ros2 run TBOT01 controller data002_room1.bin 250

```
Now the turtlebot moves from room 1 to room 4.

Here is the image for `data002_room1`,

![data002_room1](images/data002_room1.jpg?raw=true)

You can view a short video [here](https://github.com/caiks/TBOT01_ws/blob/master/data002_room1.mp4). In this simulation turtlebot leaves room 1 and goes into room 2.

We repeat for the various rooms in the turtlebot house.

Room 2,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env004.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data002_room2.bin 250

```
Room 3,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env005.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data002_room3.bin 250

```
Room 5,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env006.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data002_room5.bin 250

```
Room 5 from a different starting point,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env007.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data002_room5_2.bin 250

```
Room 2 again  from a different starting point,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env008.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data002_room2_2.bin 250

```

Now we have acquired some data, let us consider creating a *history*. The `recordListsHistoryRepa_2` function in [dev.h](https://github.com/caiks/TBOT01/blob/master/dev.h) creates a `HistoryRepa` from a list of `Record`,
```cpp
SystemHistoryRepaTuple recordListsHistoryRepa_2(int, const RecordList&);
```
The *substrate* consists of 360 `scan` *variables* with bucketed *values* and a `motor` *variable* with *values* 0,1 and 2 corresponding to left, ahead and right. It also has two *variables* derived from the `odom`, a `location` *variable* with *values* `door12`, `door13`, `door14`, `door45`, `door56`, `room1`, `room2`, `room3`, `room4`, `room5` and `room6`, and a `position` *variable* with *values* `centre`, `corner` and `side`.

We can do some analysis of the data files `data002_room1.bin`, `data002_room2.bin`, `data002_room2_2.bin`, `data002_room3.bin`, `data002_room4.bin`, `data002_room5.bin` and `data002_room5_2.bin`,
```
./main history

```
which has the following output,
```
hr->dimension: 363
hr->size: 6054
({(<scan,1>,0)},39 % 1)
({(<scan,1>,1)},892 % 1)
({(<scan,1>,2)},885 % 1)
({(<scan,1>,3)},825 % 1)
({(<scan,1>,4)},701 % 1)
({(<scan,1>,5)},685 % 1)
({(<scan,1>,6)},632 % 1)
({(<scan,1>,7)},1395 % 1)

({(<scan,180>,0)},33 % 1)
({(<scan,180>,1)},497 % 1)
({(<scan,180>,2)},777 % 1)
({(<scan,180>,3)},838 % 1)
({(<scan,180>,4)},737 % 1)
({(<scan,180>,5)},767 % 1)
({(<scan,180>,6)},757 % 1)
({(<scan,180>,7)},1648 % 1)

({(motor,0)},698 % 1)
({(motor,1)},5256 % 1)
({(motor,2)},100 % 1)

({(location,door12)},43 % 1)
({(location,door13)},13 % 1)
({(location,door14)},57 % 1)
({(location,door45)},42 % 1)
({(location,door56)},29 % 1)
({(location,room1)},1222 % 1)
({(location,room2)},572 % 1)
({(location,room3)},201 % 1)
({(location,room4)},2763 % 1)
({(location,room5)},161 % 1)
({(location,room6)},951 % 1)

({(position,centre)},1849 % 1)
({(position,corner)},890 % 1)
({(position,side)},3315 % 1)
```
There are 363 *variables* and the *size* is 6054. We show the *histograms* of the *reductions* to *variables* `<scan,1>`, `<scan,180>`, `motor`, `location` and `position`. 

We can see that with this controller the turtlebot tends to end up in the larger rooms, 1 and 4, and mainly skirts around the side of the rooms. 

Although the *history* is not very evenly spatially distributed, let us *induce* a *model* of the 360 sensor `scan` *variables*,
```
./main induce model001 4 >model001.log

```
We can view a bitmap of the averaged *slices* in the *decomposition* tree,
```
./main bitmap_model model001 

```
![model001](images/model001.jpg?raw=true)

If we looks at the *slices* at the root, we can see that objects are generally nearer on the left than on the right. This is explained by turtlebot's tendency to move in a clockwise direction skirting around the sides of the room. 

Another property of the *slices* is that some have  near objects ahead, while others have nearer objects at the sides. This is explained by turtlebot's tendency to move in straight lines until a threshold is reached (0.7 m) at which point the turtlebot stops and rotates by 30 degrees. 

The child *slices* of the near-obstacle-ahead *slice* are divided into whether there is an obstacle on the left, or on the right. TurtleBot looks to the left first, so these child *slices* are larger.

Now let us drill down into the `scan` *variables*. The `recordListsHistoryRepaRegion` function in [dev.h](https://github.com/caiks/TBOT01/blob/master/dev.h) creates a `HistoryRepa` from a random region of the `scan` *variables*,
```cpp
SystemHistoryRepaTuple recordListsHistoryRepaRegion(int, int, int, const RecordList&);
```
Let us *induce* a *model* of regions of 60 `scan` *variables*,
```
./main induce model002 4 >model002.log

./main bitmap_model model002 

```
This is the bitmap,

![model002](images/model002.jpg?raw=true)

The random-region *slices* are considerably more elemental than for the entire 360 degrees panorama.

*Model* 3 uses a *valency* of 4 buckets instead of 8. The image can be viewed [here](https://github.com/caiks/TBOT01_ws/blob/master/model003.jpg?raw=true).
The *decomposition* is narrower and deeper, but otherwise little different.

*Model* 4 goes back to a *valency* of 8, but has a smaller `wmax`. The image can be viewed [here](https://github.com/caiks/TBOT01_ws/blob/master/model004.jpg?raw=true).
Again, the *decomposition* is little different.

Now let us consider a *2-level model*. *Model 5* is *induced* from a lower *level* that consists of the *slice variables* of 12 *model 4* regions every 30 degrees,
```
./main induce model005 4 >model005.log

./main bitmap_model model005 

```
This is the bitmap,

![model005](images/model005.jpg?raw=true)

The *decomposition* is narrower and deeper than that of *model 1*. Near the root there is no *slice* for a near-object-ahead . These *alignments* are pushed downwards into the children *slices*.