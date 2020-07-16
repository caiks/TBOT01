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

cd ~/TBOT01_ws
cat data008a* >data008.bin
cat data009a* >data009.bin

```
Then build -
```
cd
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

./main analyse data002

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

./main condition model006 4 location >model006_location.log

./main condition model006 4 position >model006_position.log

./main condition model007 4 motor >model007_motor.log

./main condition model007 4 location >model007_location.log

./main condition model007 4 position >model007_position.log

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

### Sensors, motors, environment and the collision avoidance controller

In the following, the rooms are numbered 1 to 6 in the [turtlebot3 house](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#3-turtlebot3-house) from top-left to bottom-right,

![turtlebot3_house](images/turtlebot3_house.png?raw=true)

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

The [Turtlebot3Drive node](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/ros2/turtlebot3_gazebo/include/turtlebot3_gazebo/turtlebot3_drive.hpp) does simple collision avoidance. It subscribes to the lidar `scan` and odometry `odom` topics, and publishes to the motor `cmd_vel` topic. It runs a timer every 10 ms which calls a callback where the direction is decided. The `scan` data is first checked at 0 deg. If there is an obstacle ahead it turns right. If there is no obstacle ahead the `scan` data is checked at 30 deg. If there is an obstacle to the left it turns right. If there is no obstacle ahead nor to the left, the `scan` data is checked at 330 deg. If there is an obstacle to the right it turns left. If there are no obstacles ahead, left or right, it drives straight ahead. Once the direction is decided a `geometry_msgs::msg::Twist` is published to `cmd_vel` either with (a) a linear motion at 0.7 m/s, or (b) a rotation clockwise or anti-clockwise at 1.5 rad/s. While rotating the controller waits until the orientation has changed by 30 deg before the direction is decided again.

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
./main analyse data002

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

({(motor,0)},100 % 1)
({(motor,1)},5256 % 1)
({(motor,2)},698 % 1)

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

The `scan` *values* are rarely in the first bucket, which is up to 0.5 m, because this is less than the collision avoidance range of 0.7 m. The last bucket, which includes infinity, is the largest. The nearer buckets of `<scan,180>` are smaller than those of `<scan,1>` and the further buckets are larger, because the turtlebot moves forward not backward. 

We can see from the `motor` *values* that the turtlebot generally moves straight ahead and that right turns are preferred to left turns.

From the `location` and `position` *values*  we can also see that with this controller the turtlebot tends to end up in the larger rooms, 1 and 4, and mainly skirts around the side of the rooms. 

### Models of the scan substrate

Although the *history* is not very evenly spatially distributed, let us *induce* a *model* of the 360 sensor `scan` *variables*,
```
./main induce model001 4 >model001.log

```
We can compare this *model* to the *models* below by using a proxy for the *size-volume scaled component size cardinality sum relative entropy* which substitutes a *scaled shuffle* for the *cartesian*. As the *shuffle* is *scaled* the *relative entropy* gradually converges, so it is a reasonable proxy for the *model likelihood*.
```
./main entropy model001 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 17842.3
```

We can view a bitmap of the averaged *slices* in the *decomposition* tree,
```
./main bitmap_model model001 

```
![model001](images/model001.jpg?raw=true)

If we look at the *slices* at the root, we can see that objects are generally nearer on the left than on the right. This is explained by turtlebot's tendency to move in a clockwise direction skirting around the sides of the room. 

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

./main entropy model005 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 16878.8
```
The *likelihood* is a little less than for *model* 1.
```
./main bitmap_model model005 

```
This is the bitmap,

![model005](images/model005.jpg?raw=true)

The *decomposition* is narrower and deeper than that of *model 1*. Near the root there is no *slice* for near-object-ahead. These *alignments* are pushed downwards into the children *slices*.

### Models conditioned on location and position

Now let us *condition* *models* on the labels `motor`, `location` and `position`, given the `scan` *substrate*,
```
./main condition model006 4 motor >model006_motor.log

./main entropy model006_motor 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 4532.53

./main condition model006 4 location >model006_location.log

./main entropy model006_location 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 6729.56

./main condition model006 4 position >model006_position.log

./main entropy model006_position 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 5625.21
```
All of these *models* are run to zero *label entropy*. That means that if they were applied to the training *history* they would have 100% prediction accuracy. The *likelihoods* of the *conditioned models* are all considerably lower than the *induced models*. Note that `location` is the most complex label, so perhaps it captures more of the *alignments*.

Now let us run the same set of *conditioners* on a *level* that consists of the *slice variables* of 12 *model 4* regions every 30 degrees (which is the same *underlying level* in *model 5* above),
```
./main condition model007 4 motor >model007_motor.log

./main entropy model007_motor 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 8118.53

./main condition model007 4 location >model007_location.log

./main entropy model007_location 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 14054.9

./main condition model007 4 position >model007_position.log

./main entropy model007_position 10
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 13948.5
```
All of these also run to zero *label entropy* but require more *fuds* to do so. For example, to predict `location` *model 6* requires 421 *fuds* but *model 7* requires 626. From the point of view of these labels, the original *substrate* is more *causal* than the random region *level*. However, the *likelihoods* of the *models conditioned* on *underlying* regional *induced models* are all higher than those of the *models conditioned* directly on the *substrate*.

### Location and position observer

Now let us use the *models* we have created to make guesses about the `location` and `position` in a ROS node that observes the turtlebot at it moves around the turtlebot house in the gazebo simulation. The `TBOT01` [observer](https://github.com/caiks/TBOT01/blob/master/observer.h) node is given a *model*, a label *variable* and a observe interval. At each observation it *applies* the *model* to the current *event* to determine its *slice*. The prediction of the label is the most common *value* of the label *variable* in the `data002` *history's slice*. The prediction is reported along with the actual *value*, calculated from the current *event's* odometry, and a running average of the matches is calculated.

Let us consider *models* `model006_location` and `model006_position` which are *conditioned* on the label given the `scan` *substrate*. The following are run in separate shells,
```
ros2 run TBOT01 controller data.bin 250

ros2 run TBOT01 observer model006_location location 2500

ros2 run TBOT01 observer model006_position position 2500

gazebo -u ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
The turtlebot is allowed to run around for a while.

This is the output for `location`,
```
room4    room4   match   100.000000
room4    room4   match   100.000000
room4    room4   match   100.000000
room4    room4   match   100.000000
room4    unknown fail    80.000000
room4    room4   match   83.333333
room4    room4   match   85.714286
room4    room5   fail    75.000000
room4    room4   match   77.777778
room4    room4   match   80.000000
room4    room4   match   81.818182
room4    room4   match   83.333333
room4    room4   match   84.615385
room4    unknown fail    78.571429
room4    room4   match   80.000000
room4    room4   match   81.250000
room4    room4   match   82.352941
room4    room4   match   83.333333
room4    room4   match   84.210526
room4    room4   match   85.000000
room4    room4   match   85.714286
room4    unknown fail    81.818182
room4    room4   match   82.608696
room4    room4   match   83.333333
room4    room4   match   84.000000
room4    room1   fail    80.769231
room4    room4   match   81.481481
room4    room4   match   82.142857
room4    room4   match   82.758621
room4    room4   match   83.333333
room4    room1   fail    80.645161
room4    room4   match   81.250000
```
We can see that `model006` turtlebot is quite good at guessing that it is in room 4. It sometimes mistakes it for another large room, room 1. In this run, the turtlebot stayed in room 4.

This is the output for `position`,
```
centre   centre  match   100.000000
centre   centre  match   100.000000
side     side    match   100.000000
side     corner  fail    75.000000
side     centre  fail    60.000000
centre   centre  match   66.666667
centre   centre  match   71.428571
side     side    match   75.000000
side     centre  fail    66.666667
side     side    match   70.000000
centre   centre  match   72.727273
centre   centre  match   75.000000
side     side    match   76.923077
side     side    match   78.571429
side     unknown fail    73.333333
centre   centre  match   75.000000
centre   side    fail    70.588235
centre   centre  match   72.222222
centre   side    fail    68.421053
centre   side    fail    65.000000
centre   centre  match   66.666667
centre   side    fail    63.636364
side     side    match   65.217391
side     side    match   66.666667
side     side    match   68.000000
corner   corner  match   69.230769
side     side    match   70.370370
side     side    match   71.428571
side     side    match   72.413793
centre   centre  match   73.333333
centre   centre  match   74.193548
centre   centre  match   75.000000
centre   centre  match   75.757576
```
`model006` turtlebot is also quite good at guessing its `position`.

Now let us do another run starting in room 1,
```
gazebo -u ~/turtlebot3_ws/src/TBOT01_ws/env003.model -s libgazebo_ros_init.so

```
with `location` output,
```
room1    room2   fail    0.000000
room1    room4   fail    0.000000
room1    room1   match   33.333333
room1    room4   fail    25.000000
room1    room4   fail    20.000000
room1    room2   fail    16.666667
room1    room4   fail    14.285714
room1    room1   match   25.000000
door12   door12  match   33.333333
room1    room1   match   40.000000
room1    room1   match   45.454545
room1    room1   match   50.000000
room1    room4   fail    46.153846
room1    unknown fail    42.857143
room1    room1   match   46.666667
room1    room4   fail    43.750000
room1    room4   fail    41.176471
room1    room4   fail    38.888889
room1    room4   fail    36.842105
room1    room4   fail    35.000000
room1    room1   match   38.095238
```
`model006` turtlebot is not so good at guessing that it is in room 1, often mistaking it for room 4. It matches `door12` as it approaches.

This is the `position` output,
```
centre   centre  match   100.000000
centre   side    fail    50.000000
centre   centre  match   66.666667
centre   unknown fail    50.000000
centre   centre  match   60.000000
centre   centre  match   66.666667
side     side    match   71.428571
corner   side    fail    62.500000
side     side    match   66.666667
side     side    match   70.000000
centre   centre  match   72.727273
centre   centre  match   75.000000
centre   centre  match   76.923077
centre   centre  match   78.571429
side     side    match   80.000000
side     side    match   81.250000
side     side    match   82.352941
centre   centre  match   83.333333
side     side    match   84.210526
side     side    match   85.000000
side     side    match   85.714286
centre   side    fail    81.818182
side     centre  fail    78.2608
```
This is very similar to the results for room 4.

Now let us do another run starting in room 2,
```
gazebo -u ~/turtlebot3_ws/src/TBOT01_ws/env004.model -s libgazebo_ros_init.so

```
with `location` output,
```
room2    unknown fail    0.000000
room2    room4   fail    0.000000
room2    room2   match   33.333333
room2    room2   match   50.000000
room2    room2   match   60.000000
room2    room6   fail    50.000000
room2    room3   fail    42.857143
room2    room2   match   50.000000
room2    room3   fail    44.444444
room2    room6   fail    40.000000
room2    room2   match   45.454545
room2    unknown fail    41.666667
room2    room2   match   46.153846
room2    room2   match   50.000000
room2    room2   match   53.333333
room2    room3   fail    50.000000
room2    room2   match   52.941176
room2    room2   match   55.555556
room2    door12  fail    52.631579
room2    unknown fail    50.000000
room2    room4   fail    47.619048
room2    room2   match   50.000000
room2    room2   match   52.173913
```
`model006` turtlebot is better at recognising room 2 than room 1, but not as good as room 4. 

This is the `position` output,
```
corner   centre  fail    0.000000
corner   corner  match   50.000000
side     centre  fail    33.333333
corner   corner  match   50.000000
corner   corner  match   60.000000
side     side    match   66.666667
side     side    match   71.428571
centre   centre  match   75.000000
side     unknown fail    66.666667
side     side    match   70.000000
side     centre  fail    63.636364
centre   centre  match   66.666667
centre   centre  match   69.230769
corner   corner  match   71.428571
corner   corner  match   73.333333
side     side    match   75.000000
side     side    match   76.470588
side     centre  fail    72.222222
centre   centre  match   73.684211
centre   centre  match   75.000000
side     side    match   76.190476
corner   corner  match   77.272727
side     corner  fail    73.913043
side     side    match   75.000000
```
This is very similar to the results for rooms 4 and room 1.

Now let us do another run starting in room 3,
```
gazebo -u ~/turtlebot3_ws/src/TBOT01_ws/env005.model -s libgazebo_ros_init.so

```
with `location` output,
```
room3    room4   fail    0.000000
room3    unknown fail    0.000000
room3    room2   fail    0.000000
room3    room6   fail    0.000000
room3    unknown fail    0.000000
room3    room3   match   16.666667
room3    unknown fail    14.285714
room3    door14  fail    12.500000
door13   unknown fail    11.111111
room1    room1   match   20.000000
...
```
The turtlebot moves out of room 3 into room 1 after a few seconds.
`model006` turtlebot is poor at recognising room 3. This is because there are only 201 *events* in `data002` in room 3. Compare this to 572 *events* in room 2, 1222 *events* in  room 1 and 2763 *events* in 4. 

This is the `position` output,
```
side     unknown fail    0.000000
side     unknown fail    0.000000
corner   corner  match   33.333333
side     corner  fail    25.000000
side     side    match   40.000000
side     centre  fail    33.333333
centre   centre  match   42.857143
corner   centre  fail    37.500000
corner   corner  match   44.444444
corner   side    fail    40.000000
...
```
The `position` results are lower than for the other rooms.

The final run starts in room 5,
```
gazebo -u ~/turtlebot3_ws/src/TBOT01_ws/env006.model -s libgazebo_ros_init.so

```
with `location` output,
```
room5    room5   match   100.000000
room5    room4   fail    50.000000
door56   unknown fail    33.333333
room5    unknown fail    25.000000
room5    room4   fail    20.000000
room5    unknown fail    16.666667
room5    room5   match   28.571429
room5    room5   match   37.500000
room5    unknown fail    33.333333
room5    door45  fail    30.000000
room5    room5   match   36.363636
door45   unknown fail    33.333333
room4    room1   fail    30.769231
...
```
The turtlebot moves out of room 5 into room 4 after a few seconds.
`model006` turtlebot is poor at recognising room 5, but better than room 3. Again, this is because there are only 161  *events* in `data002` in room 5. 

This is the `position` output,
```
side     unknown fail    0.000000
corner   corner  match   50.000000
side     centre  fail    33.333333
side     side    match   50.000000
side     corner  fail    40.000000
side     unknown fail    33.333333
side     side    match   42.857143
corner   corner  match   50.000000
corner   corner  match   55.555556
side     unknown fail    50.000000
side     side    match   54.545455
side     centre  fail    50.000000
side     centre  fail    46.153846
...
```
The `position` results are similar to those of room 3.

### Modelling with an unbiased controller

Now let us see if we can encourage the turtlebot to travel between rooms by removing the bias to the right. We will set an interval that alternates the bias.

The simulation was restarted in room 4,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
and the controller re-run, alternating the bias every 1000 ms,
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data003.bin 250 1000

```
Now the turtlebot moves between rooms more freely, although it tends to be a little indecisive in corners, especially around the rubbish bin in room 3. After 12 minutes it has visited all of them. 

Note that because the left-right preference switches randomly (with the given timescale) the runs are not reproducible at all after the first turn.

Again we can examine the statistics,
```
./main stats data003
rr->size(): 13381
...
```
The turtlebot ran for around 55 minutes.

Let us compare the analysis of the `data002` and `data003` data files,
```
./main analyse data003

hr->dimension: 363
hr->size: 13381
({(<scan,1>,0)},89 % 1)
({(<scan,1>,1)},2710 % 1)
({(<scan,1>,2)},1928 % 1)
({(<scan,1>,3)},1642 % 1)
({(<scan,1>,4)},1435 % 1)
({(<scan,1>,5)},1234 % 1)
({(<scan,1>,6)},1156 % 1)
({(<scan,1>,7)},3187 % 1)

({(<scan,180>,0)},81 % 1)
({(<scan,180>,1)},1280 % 1)
({(<scan,180>,2)},1549 % 1)
({(<scan,180>,3)},1648 % 1)
({(<scan,180>,4)},1411 % 1)
({(<scan,180>,5)},1363 % 1)
({(<scan,180>,6)},1229 % 1)
({(<scan,180>,7)},4820 % 1)

({(motor,0)},1140 % 1)
({(motor,1)},11096 % 1)
({(motor,2)},1145 % 1)

({(location,door12)},178 % 1)
({(location,door13)},120 % 1)
({(location,door14)},196 % 1)
({(location,door45)},86 % 1)
({(location,door56)},191 % 1)
({(location,room1)},3852 % 1)
({(location,room2)},1307 % 1)
({(location,room3)},925 % 1)
({(location,room4)},3796 % 1)
({(location,room5)},977 % 1)
({(location,room6)},1753 % 1)

({(position,centre)},3478 % 1)
({(position,corner)},3028 % 1)
({(position,side)},6875 % 1)
```
The distribution of the `scan` *values* is very similar to the previous dataset.

We can see from the `motor` *values* that the turtlebot generally moves straight ahead as before but now there is no bias for right turns over left turns.

From the `location` and `position` *values*  we can also see that with this controller the turtlebot still tends to end up in the larger rooms, 1 and 4, but now there is no bias to room 4 and it spends a larger proportion of its time in the smaller rooms. It still mainly skirts around the side of the rooms, but is more in the corner than before because of its slight indecisiveness. It also seems to have passed through the doorways more often.

Now let us *induce* a *model* from the new dataset `data003`. *Model* 9 uses the same parameters as *model* 1,
```
./main induce model009 4 >model009.log

./main entropy model009 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 40868.9
```
The *size-shuffle sum relative entropy* is not comparable because the *size* of the *history* is different, but we can compare *model* 1 using `data003` instead of `data002`,
```
./main entropy model001 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 37847.2
```
Unsurprisingly, *model* 9 is the more *likely model* given `data003`.

Here is the bitmap
```
./main bitmap_model model009 5 data003
```
![model009](images/model009.jpg?raw=true)

Now the bias to the right has disappeared. The main root *alignment* depends on whether there is a near obstacle to either side or a near obstacle ahead or no near obstacle.

We can look at the effect of increasing `fmax` from 127 to 512 in *model* 10,
```
./main induce model010 4 >model010.log

./main entropy model010 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 41970.1
```
We can see that there is only a small increase in the *likelihood*.

Now let us *induce* a regional *model* from the new dataset `data003`. *Model* 11 uses the same parameters as *model* 4,
```
./main induce model011 4 >model011.log

```
Now let us create the *2-level model*. *Model 12* is *induced* from a lower *level* that consists of the *slice variables* of 12 *model 11* regions every 30 degrees. It has the same parameters as *model* 5,
```
./main induce model012 4 >model012.log

./main entropy model012 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 40676.3
```
The *likelihood* is a little more than for *model* 9.

This is the bitmap,
```
./main bitmap_model model012 5 data003

```
![model012](images/model012.jpg?raw=true)

Now let us *condition* *models* on the labels `motor`, `location` and `position`, given the `scan` *substrate*. *Model* 13 has the same parameters as *model* 6,
```
./main condition model013 4 motor >model013_motor.log

./main entropy model013_motor 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 12257.1

./main condition model013 4 location >model013_location.log

./main entropy model013_location 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 18075.9

./main condition model013 4 position >model013_position.log

./main entropy model013_position 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 13424.3
```
Again, all of these *models* are run to zero *label entropy*. The *likelihoods* of the *conditioned models* are all considerably lower than the *induced models*. Again, `location` is the most complex label, because it probably captures more of the *alignments*.

Now let us run the same set of *conditioners* on a *level* that consists of the *slice variables* of 12 *model 11* regions every 30 degrees (which is the same *underlying level* in *model 12* above). *Model* 14 has the same parameters as *model* 7,
```
./main condition model014 4 motor >model014_motor.log

./main entropy model014_motor 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 28086.1

./main condition model014 4 location >model014_location.log

./main entropy model014_location 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 39993.4

./main condition model014 4 position >model014_position.log

./main entropy model014_position 10 data003
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 30755.2
```

The *likelihood* of the `location` *conditioned model* 14 is almost the same as the *induced model* 12. We would therefore expect that *model* 14 will have a similar label accuracy to the *substrate conditioned model* 13, but be generally more robust.

Now let us test the `location` and `position` accuracy of the *induced* and *conditioned models* derived from `data003`. 

First we shall run *model* 13, which is *conditioned* on the *substrate*. The simulation was restarted in room 4,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

cd ~/turtlebot3_ws/src/TBOT01_ws

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
and the controller re-run, alternating the bias every 1000 ms,
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data004_01.bin 250 1000

ros2 run TBOT01 observer model013_location location 2500

ros2 run TBOT01 observer model013_position position 2500

```
After around an hour the turtlebot has visited every room. The last few lines from the `location` observer are
```
room5    room5   match   59.247889
room5    room4   fail    59.202454
room5    room3   fail    59.157088
room5    room5   match   59.188361
room5    room6   fail    59.143076
room5    room6   fail    59.097859
room5    room4   fail    59.052712
door45   room2   fail    59.007634
room4    room4   match   59.038902
room4    room1   fail    58.993902
room4    room1   fail    58.948972
room4    room1   fail    58.904110
room4    room1   fail    58.859316
room4    room1   fail    58.814590
room4    room1   fail    58.769932
room4    room4   match   58.801214
room4    room4   match   58.832449
room4    room4   match   58.863636
room4    room4   match   58.894777
```
The *model* 13 `location` accuracy is around 59%.

Similarly for the `position`,
```
corner   corner  match   72.371450
corner   corner  match   72.392638
corner   unknown fail    72.337165
corner   unknown fail    72.281776
corner   side    fail    72.226473
centre   corner  fail    72.171254
side     centre  fail    72.116119
side     centre  fail    72.061069
side     centre  fail    72.006102
centre   side    fail    71.951220
centre   side    fail    71.896420
centre   centre  match   71.917808
side     side    match   71.939163
side     side    match   71.960486
side     side    match   71.981777
side     side    match   72.003035
side     corner  fail    71.948446
side     side    match   71.969697
corner   corner  match   71.990916
```
The *model* 13 `position` accuracy is around 72%.

Repeating for *model* 14,
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data004_02.bin 250 1000

ros2 run TBOT01 observer model014_location location 2500 data003
...
room1    room4   fail    63.630184

ros2 run TBOT01 observer model014_position position 2500 data003
...
centre   centre  match   79.918312
```
The *model* 14 `location` accuracy is a little higher at around 64% and the `position` accuracy is higher at around 80%.

We can compare the *conditioned models*, 13 and 14, above to the three *induced models* 9, 10 and 12,

```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data004_03.bin 250 1000

ros2 run TBOT01 observer model009 location 2500 data003
...
room4    room1   fail    39.401294

ros2 run TBOT01 observer model009 position 2500 data003
...
centre   centre  match   59.902991

```
The *model* 9 `location` accuracy is a considerably lower than either of the *conditioned models* at around 39% and the `position` accuracy is also lower at around 60%.

```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data004_04.bin 250 1000

ros2 run TBOT01 observer model010 location 2500 data003
...
room2    room1   fail    41.637990

ros2 run TBOT01 observer model010 position 2500 data003
...
corner   corner  match   56.129477

```
The *model* 10 is the same as *model* 9 but with a greater `fmax`. Its `location` accuracy is a slightly higher at around 42% and the `position` accuracy is  lower at around 56%.
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data004_05.bin 250 1000

ros2 run TBOT01 observer model012 location 2500 data003
...
room4    room5   fail    42.396313

ros2 run TBOT01 observer model012 position 2500 data003
...
side     centre  fail    62.720984
```
*Model* 12 has a higher *likelihood* than *models* 9 or 10 and a slightly higher accuracy. Its `location` accuracy is around 42% and the `position` accuracy is around 63%.

Overall the results are as follows -

Model|Type|Dataset|Location %|Position %
---|---|---|---|---
9|induced|3|39|60
10|induced|3|42|56
12|induced|3|42|63
13|conditioned|3|59|72
14|conditioned|3|64|80

Let us see if we can improve on *model* 14 with the additional data in `data004`. 

*Model 15* has the same configuration as *model 11* except its *history* consists of the union of the `data003` and `data004` records, and `fmax` is increased from 127 to 384,

```
cd ~/TBOT01_ws
./main induce model015 4 >model015.log

```
Let us compare its *likelihood* to that of *model* 11,
```
./main entropy_region model015 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 968762

./main entropy_region model011 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 933468
```
*Model 16* is *induced* from a lower *level* that consists of the *slice variables* of 12 *model 15* regions every 30 degrees. It has the same parameters as *model* 14 except its *history* consists of the union of the `data003` and `data004` records, and `fmax` is increased from 1024 to 4096,

```
cd ~/TBOT01_ws
./main condition model016 8 location >model016_location.log

./main condition model016 8 position >model016_position.log

```
Even with an `fmax` of 4096, neither *conditioned model* is completely resolved. 

When we compare the *likelihoods* to those of *model* 14, there is little change,
```
./main entropy model016_location 10 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 235340

./main entropy model014_location 10 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 239912


./main entropy model016_position 10 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 197310

./main entropy model014_position 10 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 184506
```
Now, however, the label accuracy has improved considerably,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

cd ~/turtlebot3_ws/src/TBOT01_ws

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```

```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data.bin 250 1000

ros2 run TBOT01 observer model016_location location 2500 data004
...
room6    room6   match   85.122898

ros2 run TBOT01 observer model016_position position 2500 data004
...
side     side    match   88.745149
```
Now let us *induce* a *model* to a similar depth for comparison. *Model* 17 has the same parameters as *model* 12, but the *underlying model* is *model* 15 and the `fmax` is increased to 4096,
```
cd ~/TBOT01_ws
./main induce model017 8 >model017.log

./main entropy model017 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 108041
```
The *model* 17 has 72652 *transforms* so its *size-shuffle sum relative entropy* is calculated here without *shuffle scaling* to minimise the memory required. Compare the other *models* with the same *scaling* and *history*,
```
./main entropy model009 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 98888.3

./main entropy model010 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 100474

./main entropy model012 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 99696.5

./main entropy model013_location 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 45334.5

./main entropy model014_location 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 97843.5

./main entropy model016_location 1 data004
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 97093.9
```
Now we find the label accuracy for *model* 17,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

cd ~/turtlebot3_ws/src/TBOT01_ws

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```

```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data.bin 250 1000

ros2 run TBOT01 observer model017 location 2500 data004
...
door56   room5   fail    58.469388

ros2 run TBOT01 observer model017 position 2500 data004
...
side     side    match   68.536461
```
We can update our table,

Model|Type|Underlying|Fmax|Dataset|Likelihood|Location %|Position %
---|---|---|---|---|---|---|---
9|induced|substrate|127|3|98,888|39|60
10|induced|substrate|512|3|100,474|42|56
12|induced|model 11|127|3|99,696|42|63
17|induced|model 15|4096|4|108,041|58|69
13|conditioned|substrate|1024|3|45,334|59|72
14|conditioned|model 11|1024|3|97,843|64|80
16|conditioned|model 15|4096|4|97,093|85|89

We can see that *induced model* 17 is considerably more accurate than *induced model* 12, but it is still less accurate than any of the *conditioned models*. The *induced models* are all more *likely* than any of the *conditioned models*, however. The larger *induced models* with 2 *levels* have the greatest *likelihoods*, but the increase in *likelihood* is small which suggests that most of the interesting *alignments* have already been captured.

### Timewise frames

*Model* 16 was *conditioned* on 81261 *events* and has 28315 *transforms*. No doubt larger *models* *conditioned* on more *history* would incrementally increase the label accuracy, but for now let us consider increasing the *substrate* instead with timewise *frames*.

To do that we must first select which of the past records will comprise the short term memory, i.e. the *frames*. Let us image the first 60 *events* in `data003`,
```
cd ~/TBOT01_ws
./main bitmap data003 10 0 59

```
Each *event* is scaled vertically by 10 pixels,

![data003](images/data003.bmp?raw=true)

In these 12 seconds, the turtlebot starts in the room 4, facing to the right, turns to the left by 30 degrees and then moves towards the corridor to room 1,

1 second|12 seconds
---|---
![env002_1s](images/env002_1s.jpg?raw=true)|![env002_12s](images/env002_12s.jpg?raw=true)

We can *apply* a *model* to the 60 *events*  and average the corresponding *slice* to see the *history* as turtlebot 'sees' it. For example, if we *apply* *model* 9 with the *model* *history* `data003`,
```
./main observe_bitmap data003 model009 data003 10 0 59

```
the turtlebot 'sees' this -

![data003 model009 data003](images/data003_model009_data003.bmp?raw=true)

Let us do this for the other *models* and display them side by side,
```
./main observe_bitmap data003 model010 data003 10 0 59
./main observe_bitmap data003 model012 data003 10 0 59
./main observe_bitmap data003 model013_location data003 10 0 59
./main observe_bitmap data003 model014_location data003 10 0 59
./main observe_bitmap data003 model016_location data004 10 0 59
./main observe_bitmap data003 model017 data004 10 0 59

```
First, the *induced models*,

data003|model010|model012|model017
---|---|---|---
![data003](images/data003.bmp?raw=true)|![data003 model010 data003](images/data003_model010_data003.bmp?raw=true)|![data003 model012 data003](images/data003_model012_data003.bmp?raw=true)|![data003 model017 data004](images/data003_model017_data004.bmp?raw=true)

Then, the *conditioned models*,

data003|model013|model014|model016
---|---|---|---
![data003](images/data003.bmp?raw=true)|![data003 model013_location data003](images/data003_model013_location_data003.bmp?raw=true)|![data003 model014_location data003](images/data003_model014_location_data003.bmp?raw=true)|![data003 model016_location data004](images/data003_model016_location_data004.bmp?raw=true)

Note that the *induced model* 12 has only 127 *fuds*, which explains its blurriness compared to its neighbours. Perhaps the most literal of the images that of *incduced model* 17. Its counterpart, *conditioned model* 16 is considerably smoother, which suggests that the detail is not relevant to the `location` label. Compare it to *conditioned model* 13, a one *level model* *depending* directly on the *substrate*, which superficially captures a lot of the detail but is less accurate and frequently is mistaken about individual *events*.

Let us observe the *slice variables* and the label matches for each *event*. First *induced model* 17,
```
./main observe data003 model017 data004 location 0 59

```
event|variable|location|guess|match?
---|---|---|---|---
0|<<<0,1123>,s>,2>|room4|room1|fail
1|<<<0,2053>,s>,2>|room4|room4|match
2|<<<0,3055>,s>,2>|room4|room4|match
3|<<<0,3055>,s>,2>|room4|room4|match
4|<<<0,3055>,s>,2>|room4|room4|match
5|<<<0,2029>,s>,2>|room4|room4|match
6|<<<0,2029>,s>,2>|room4|room4|match
7|<<<0,2029>,s>,2>|room4|room4|match
8|<<<0,1384>,s>,2>|room4|room4|match
9|<<<0,1384>,s>,2>|room4|room4|match
10|<<<0,1384>,s>,2>|room4|room4|match
11|<<<0,2380>,s>,1>|room4|room1|fail
12|<<<0,1807>,s>,2>|room4|room4|match
13|<<<0,1807>,s>,2>|room4|room4|match
14|<<<0,1807>,s>,2>|room4|room4|match
15|<<<0,1807>,s>,2>|room4|room4|match
16|<<<0,1807>,s>,2>|room4|room4|match
17|<<<0,2463>,s>,1>|room4|room4|match
18|<<<0,2463>,s>,1>|room4|room4|match
19|<<<0,2463>,s>,2>|room4|room4|match
20|<<<0,2463>,s>,3>|room4|room4|match
21|<<<0,3023>,s>,1>|room4|room4|match
22|<<<0,2478>,s>,2>|room4|room4|match
23|<<<0,2478>,s>,2>|room4|room4|match
24|<<<0,2478>,s>,2>|room4|room4|match
25|<<<0,162>,s>,1>|room4|room1|fail
26|<<<0,2995>,s>,1>|room4|room4|match
27|<<<0,2995>,s>,1>|room4|room4|match
28|<<<0,2611>,s>,2>|room4|room4|match
29|<<<0,3318>,s>,1>|room4|room4|match
30|<<<0,2320>,s>,2>|room4|room4|match
31|<<<0,2320>,s>,2>|room4|room4|match
32|<<<0,3067>,s>,2>|room4|door13|fail
33|<<<0,3508>,s>,1>|room4|room4|match
34|<<<0,3508>,s>,1>|room4|room4|match
35|<<<0,3508>,s>,2>|room4|room4|match
36|<<<0,1278>,s>,2>|room4|room4|match
37|<<<0,4077>,s>,1>|room4|room4|match
38|<<<0,4077>,s>,1>|room4|room4|match
39|<<<0,1465>,s>,2>|room4|room4|match
40|<<<0,844>,s>,2>|room4|room4|match
41|<<<0,844>,s>,2>|room4|room4|match
42|<<<0,1517>,s>,2>|room4|room4|match
43|<<<0,3702>,s>,2>|room4|room4|match
44|<<<0,3702>,s>,1>|room4|room1|fail
45|<<<0,3702>,s>,1>|room4|room1|fail
46|<<<0,1835>,s>,1>|room4|room4|match
47|<<<0,428>,s>,2>|room4|room2|fail
48|<<<0,2492>,s>,2>|room4|room4|match
49|<<<0,2737>,s>,1>|room4|room5|fail
50|<<<0,2052>,s>,1>|room4|room4|match
51|<<<0,3592>,s>,1>|room4|room4|match
52|<<<0,3592>,s>,1>|room4|room4|match
53|<<<0,3592>,s>,1>|room4|room4|match
54|<<<0,2837>,s>,1>|room4|room4|match
55|<<<0,2641>,s>,1>|room4|room4|match
56|<<<0,4086>,s>,1>|room4|room4|match
57|<<<0,3325>,s>,2>|room4|room4|match
58|<<<0,2488>,s>,3>|room4|room4|match
59|<<<0,2488>,s>,3>|room4|room4|match

Of these 60 *events* there are 38 unique consecutive *slices*, which suggests a *frame* every 2 *events* or 0.5 seconds.

Now *conditioned model* 16,
```
./main observe data003 model016_location data004 location 0 59

```
event|variable|location|guess|match?
---|---|---|---|---
0|<<<1,445>,s>,2>|room4|room4|match
1|<<<1,3146>,s>,1>|room4|room4|match
2|<<<1,2853>,s>,2>|room4|room4|match
3|<<<1,3948>,s>,1>|room4|room4|match
4|<<<1,3948>,s>,1>|room4|room4|match
5|<<<1,3948>,s>,1>|room4|room4|match
6|<<<1,3948>,s>,1>|room4|room4|match
7|<<<1,3948>,s>,1>|room4|room4|match
8|<<<1,3948>,s>,1>|room4|room4|match
9|<<<1,3948>,s>,1>|room4|room4|match
10|<<<1,3948>,s>,1>|room4|room4|match
11|<<<1,3948>,s>,1>|room4|room4|match
12|<<<1,2105>,s>,2>|room4|room4|match
13|<<<1,2850>,s>,2>|room4|room4|match
14|<<<1,3532>,s>,1>|room4|room4|match
15|<<<1,3532>,s>,1>|room4|room4|match
16|<<<1,3532>,s>,1>|room4|room4|match
17|<<<1,3689>,s>,1>|room4|room4|match
18|<<<1,3689>,s>,1>|room4|room4|match
19|<<<1,1624>,s>,2>|room4|room4|match
20|<<<1,1624>,s>,2>|room4|room4|match
21|<<<1,3111>,s>,2>|room4|room4|match
22|<<<1,3373>,s>,1>|room4|room4|match
23|<<<1,3373>,s>,1>|room4|room4|match
24|<<<1,3373>,s>,1>|room4|room4|match
25|<<<1,3373>,s>,1>|room4|room4|match
26|<<<1,2629>,s>,2>|room4|room4|match
27|<<<1,3373>,s>,1>|room4|room4|match
28|<<<1,3146>,s>,1>|room4|room4|match
29|<<<1,2876>,s>,2>|room4|room4|match
30|<<<1,790>,s>,2>|room4|room4|match
31|<<<1,790>,s>,2>|room4|room4|match
32|<<<1,790>,s>,2>|room4|room4|match
33|<<<1,790>,s>,2>|room4|room4|match
34|<<<1,790>,s>,2>|room4|room4|match
35|<<<1,914>,s>,2>|room4|room4|match
36|<<<1,3146>,s>,1>|room4|room4|match
37|<<<1,2853>,s>,2>|room4|room4|match
38|<<<1,2853>,s>,2>|room4|room4|match
39|<<<1,2853>,s>,2>|room4|room4|match
40|<<<1,2853>,s>,2>|room4|room4|match
41|<<<1,2853>,s>,2>|room4|room4|match
42|<<<1,3146>,s>,1>|room4|room4|match
43|<<<1,2940>,s>,2>|room4|room4|match
44|<<<1,790>,s>,2>|room4|room4|match
45|<<<1,2492>,s>,1>|room4|room4|match
46|<<<1,2492>,s>,1>|room4|room4|match
47|<<<1,2492>,s>,1>|room4|room4|match
48|<<<1,2492>,s>,1>|room4|room4|match
49|<<<1,4037>,s>,1>|room4|room4|match
50|<<<1,4037>,s>,1>|room4|room4|match
51|<<<1,4037>,s>,1>|room4|room4|match
52|<<<1,4037>,s>,1>|room4|room4|match
53|<<<1,4037>,s>,1>|room4|room4|match
54|<<<1,4037>,s>,1>|room4|room4|match
55|<<<1,2994>,s>,2>|room4|room4|match
56|<<<1,3923>,s>,1>|room4|room4|match
57|<<<1,1085>,s>,2>|room4|room4|match
58|<<<1,1085>,s>,2>|room4|room4|match
59|<<<1,1085>,s>,2>|room4|room4|match

Of these 60 *events* there are 27 unique consecutive *slices*, which also suggests a *frame* every 0.5 seconds. Let us check that this rate applies in general,

```
./main observe data003 model017 data004 location
...
z: 13381
slice_unique.size(): 3595
consecutive_unique_count: 7765
match_count: 10256
100.0*match_count/z: 76.646

./main observe data004_04 model017 data004 location
...
z: 14546
slice_unique.size(): 3635
consecutive_unique_count: 8581
match_count: 11031
100.0*match_count/z: 75.8353

./main observe data003 model016_location data004 location
...
z: 13381
slice_unique.size(): 2216
consecutive_unique_count: 5489
match_count: 12962
100.0*match_count/z: 96.8687

./main observe data004_04 model016_location data004 location
...
z: 14546
slice_unique.size(): 2290
consecutive_unique_count: 6199
match_count: 14015
100.0*match_count/z: 96.3495

./main observe data003 model017 data004 position
...
z: 13381
slice_unique.size(): 3595
consecutive_unique_count: 7765
match_count: 11409
00.0*match_count/z: 85.2627

./main observe data003 model016_position data004 position
...
z: 13381
slice_unique.size(): 1973
consecutive_unique_count: 6236
match_count: 13158
100.0*match_count/z: 98.3335
```
Note that the number of matches here is higher than for the table above. This is because the *models* were *induced* or *conditioned* on `data004` which includes the `data003` records.

The addition of timewise *frames* requires scaling the *models*, so before we do that let us consider the *underlying level models* *models* 11 and 15. We saw above that the *likelihoods* are not much different, but *model* 11 has only 2487 *transforms* while *model* 15 has 6685. The *derived induced models* 12 and 17 have 5600 and 72652 *transforms* respectively. Of course, this may be purely due to `fmax` increasing from 127 to 4096, but let us check this in a new *induced model* 18 that has *model* 11 for its *underlying*, but is otherwise the same as *model 17*.
```
cd ~/TBOT01_ws
./main induce model018 8 >model018.log

./main entropy model018 1 data004
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 110278

./main condition model019 8 location >model019_location.log

./main entropy model019_location 1 data004
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 106063
```
Let us also also observe the accuracy using a common test dataset `data005` that has not been used for *modelling*,

```
cd ~/TBOT01_ws
./main observe data005 model018 data004 location
...
100.0*match_count/z: 56.7202

./main observe data005 model018 data004 position
...
100.0*match_count/z: 71.6021

./main observe data005 model017 data004 location
...
100.0*match_count/z: 59.5346

./main observe data005 model017 data004 position
...
100.0*match_count/z: 71.7463

./main observe data005 model019_location data004 location
...
100.0*match_count/z: 84.7405

./main observe data005 model019_position data004 position
...
100.0*match_count/z: 89.3259

./main observe data005 model016_location data004 location
...
100.0*match_count/z: 86.285

./main observe data005 model016_position data004 position
...
100.0*match_count/z: 89.8339

./main bitmap data005 10 0 59
./main observe_bitmap data005 model018 data004 10 0 59
./main observe_bitmap data005 model017 data004 10 0 59
./main observe_bitmap data005 model019_location data004 10 0 59
./main observe_bitmap data005 model016_location data004 10 0 59

```

Model|Type|Underlying|Fmax|Dataset|Likelihood|Location %|Position %
---|---|---|---|---|---|---|---
18|induced|model 11|4096|4|110,278|57|72
17|induced|model 15|4096|4|108,041|60|72
19|conditioned|model 11|4096|4|106,063|85|89
16|conditioned|model 15|4096|4|97,093|86|90


First, the *induced models*,

data005|model018|model017
---|---|---
![data005](images/data005.bmp?raw=true)|![data005 model018 data004](images/data005_model018_data004.bmp?raw=true)|![data005 model017 data004](images/data005_model017_data004.bmp?raw=true)

Then, the *conditioned models*,

data005|model019|model016
---|---|---
![data005](images/data005.bmp?raw=true)|![data005 model019_location data004](images/data005_model019_location_data004.bmp?raw=true)|![data005 model016_location data004](images/data005_model016_location_data004.bmp?raw=true)

So the smaller *underlying model* has little effect on the accuracy and actually increases the *likelihood*.

Now let us consider timewise sequences of *frames* to see if the added *substrate* improves either *likelihood* or label accuracy. *Model* 20 is a copy of *conditioned model* 19 except that the *substrate* includes `sequence_count` *frames* of the immediately previous *events* every `sequence_interval` *events*. Let us first check that a single *frame* has the same results as for *model* 19,
```
./main condition model020 8 location 1 1 >model020_location_1_1.log

./main entropy_sequence model020_location_1_1 1 data004 1 1
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 106063

./main observe_sequence data005 model020_location_1_1 data004 location 1 1
...
100.0*match_count/z: 84.7405

./main entropy model019_location 1 data004
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 106063

./main observe data005 model019_location data004 location
...
100.0*match_count/z: 84.7405
```
This is indeed the case. Now let us present the results for *model* 20 for various sequence parameters. In addition, *induced model* 21 will be the timewise version of *induced model* 18. Also we shall compare these to two *level models*, *conditioned model* 22 and *induced model* 23, where the *underlying model* is *model* 18 rather than 12 spacewise instances of *model* 11. All will have a `sequence_interval` of 4 *events* or 1 second,

Model|Type|Underlying|Fmax|Dataset|Sequence length|Likelihood|Location %|Position %
---|---|---|---|---|---|---|---|---
18|induced|model 11|4096|4|1|110,278|57|72
21|induced|model 11|4096|4|2|110,310|52|-
21|induced|model 11|4096|4|3|109,378|50|-
23|induced|model 18|4096|4|2|109,218|53|-
19|conditioned|model 11|4096|4|1|106,063|85|89
20|conditioned|model 11|4096|4|2|105,960|83|-
20|conditioned|model 11|4096|4|3|103,220|82|-
20|conditioned|model 11|4096|4|7|103,874|80|-
22|conditioned|model 18|4096|4|2|107,855|64|-

We can see that the addition of the *values* as they were in the *events* a few seconds ago, makes little difference to the *likelihood* and in all cases reduces the `location` accuracy. Clearly the *alignments* within the *frames* are much greater than those between the *frames*, suggesting that for the `data004` *history size*, at least, the static information is more important than the dynamic. Also the dynamic information perhaps causes some over-fitting in the *conditioned models* leading to lower *likelihoods* and label accuracies.

Before we move on to consider *models* that control the `motor` *variable*, let us increase the random region *substrate* with timewise *frames* as the *underlying* in a two *level model*. *Model* 24 is a timewise version of *model* 15, with *frames* every half second for three seconds,
```
./main induce model024 8 7 2  >model024.log

```
We can view the first 60 *events*,
```
./main bitmap_reqion_sequence 7 2 10 0 59

```
The sequence of 7 *frames* has the earliest *frame* on the left and the latest on the right,

![data004_7_2](images/data004_7_2.bmp?raw=true)

We can compare the regional *likelihoods* (all with a scaling factor of 1 to make them comparable),
```
./main entropy_region model011 1 data004 1
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 93433.5

./main entropy_region model015 1 data004 1
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 97069.6

./main entropy_region_sequence model024 1 7 2
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 101416
```
Model|Type|Fmax|Dataset|Sequence length|Likelihood
---|---|---|---|---|---
11|induced|127|4|1|93,433
15|induced|384|4|1|97,069
24|induced|384|4|7|101,416

We can see that the timewise regional *model* is only a little more *likely*.

Finally let us generate *conditioned model* 25 with 12 spacewise *frames* of *underlying model* 24,
```
./main condition model025 8 location 7 2 >model025_location_7_2.log

./main entropy_sequence model025_location_7_2 1 data004 7 2
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 103393

./main observe_sequence data005 model025_location_7_2 data004 location 7 2
...
100.0*match_count/z: 73.8733

```
Model|Type|Underlying|Fmax|Dataset|Sequence length|Sequence step|Likelihood|Location %|Position %
---|---|---|---|---|---|---|---|---|---
18|induced|model 11|4096|4|1|4|110,278|57|72
21|induced|model 11|4096|4|2|4|110,310|52|-
21|induced|model 11|4096|4|3|4|109,378|50|-
23|induced|model 18|4096|4|2|4|109,218|53|-
19|conditioned|model 11|4096|4|1|4|106,063|85|89
20|conditioned|model 11|4096|4|2|4|105,960|83|-
20|conditioned|model 11|4096|4|3|4|103,220|82|-
20|conditioned|model 11|4096|4|7|4|103,874|80|-
22|conditioned|model 18|4096|4|2|4|107,855|64|-
25|conditioned|model 24|4096|4|7|2|103,393|74|-

*Model* 25 is a three *level model* with the top *level conditioned* on a spacewise *substrate* of two *level model 24 induced* on a timewise *substrate* of one *level model 11 induced* on a random region *substrate*. Compare it to *model* 22 which is a three *level model* with the top *level conditioned* on a timewise *substrate* of two *level model 18 induced* on a spacewise *substrate* of one *level model 11 induced* on a random region *substrate*. *Model* 25 has a lower *likelihood* but a higher accuracy than *model* 22. So *model* 25 is intermediate between *model* 22 and *model* 20, which is the two *level conditioned* on a spacetimewise *substrate* of one *level model 11 induced* on a random region *substrate*. This confirms that the static information is more important than the dynamic.

### Motor actions

Given that we wish now to move on to motor actions, we are interested in the sequence of *events* in order to determine future labels for each *event*. It was found, however, that the existing collision avoidance algorithm was not enough to prevent occasional crashes in the previous datasets, making it difficult to obtain long sequences. The crashes were due to collisions with the open shelves and bookcases. The solution was to simply turn the shelves and bookcases around so that they were closed and thus detectable by the turtlebot's lidar. The new environment is `env009`.

In addition, we have added a random turn interval to the controller. If the turtlebot is going straight it will randomly turn left or right on a timescale on the order of the turn interval.

We ran the simulation with the new environment,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT01_ws/gazebo_models

cd ~/turtlebot3_ws/src/TBOT01_ws

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```
and ran the new controller with a random turn interval of 5000 ms -
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data008.bin 250 5000 5000 

```
Note that the bias interval has been increased to 5000 ms to reduce dither in the corners.

The turtlebot ran for 5.5 hours to create a test dataset `data008`,
```
cd ~/TBOT01_ws

./main analyse data008
hr->dimension: 363
hr->size: 78612
({(<scan,1>,0)},283 % 1)
({(<scan,1>,1)},15049 % 1)
({(<scan,1>,2)},12504 % 1)
({(<scan,1>,3)},9952 % 1)
({(<scan,1>,4)},7933 % 1)
({(<scan,1>,5)},6710 % 1)
({(<scan,1>,6)},5842 % 1)
({(<scan,1>,7)},20339 % 1)

({(<scan,180>,0)},398 % 1)
({(<scan,180>,1)},8726 % 1)
({(<scan,180>,2)},11379 % 1)
({(<scan,180>,3)},10790 % 1)
({(<scan,180>,4)},8291 % 1)
({(<scan,180>,5)},7373 % 1)
({(<scan,180>,6)},6584 % 1)
({(<scan,180>,7)},25071 % 1)

({(motor,0)},8255 % 1)
({(motor,1)},62301 % 1)
({(motor,2)},8056 % 1)

({(location,door12)},580 % 1)
({(location,door13)},524 % 1)
({(location,door14)},1023 % 1)
({(location,door45)},899 % 1)
({(location,door56)},1805 % 1)
({(location,room1)},14793 % 1)
({(location,room2)},4446 % 1)
({(location,room3)},3478 % 1)
({(location,room4)},26175 % 1)
({(location,room5)},10508 % 1)
({(location,room6)},14381 % 1)

({(position,centre)},19756 % 1)
({(position,corner)},16408 % 1)
({(position,side)},42448 % 1)
```
The turtlebot spent 65% of its time in rooms 4, 5 and 6.

Compare to `data003`,
```
./main analyse data003
hr->dimension: 363
hr->size: 13381
...
({(motor,0)},1140 % 1)
({(motor,1)},11096 % 1)
({(motor,2)},1145 % 1)

({(location,door12)},178 % 1)
({(location,door13)},120 % 1)
({(location,door14)},196 % 1)
({(location,door45)},86 % 1)
({(location,door56)},191 % 1)
({(location,room1)},3852 % 1)
({(location,room2)},1307 % 1)
({(location,room3)},925 % 1)
({(location,room4)},3796 % 1)
({(location,room5)},977 % 1)
({(location,room6)},1753 % 1)

({(position,centre)},3478 % 1)
({(position,corner)},3028 % 1)
({(position,side)},6875 % 1)
```
In `data009` 3% more time is spent in a turn but 2% less time is spent in a corner.

We ran the turtlebot again to create a training dataset, `data009`, this time for 12 hours -
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data009.bin 250 5000 5000 

```
Now the turtlebot spends 48% of its time in rooms 4, 5 and 6 -
```
cd ~/TBOT01_ws

./main analyse data009
hr->dimension: 363
hr->size: 172301
({(<scan,1>,0)},692 % 1)
({(<scan,1>,1)},33376 % 1)
({(<scan,1>,2)},27585 % 1)
({(<scan,1>,3)},22370 % 1)
({(<scan,1>,4)},17382 % 1)
({(<scan,1>,5)},14909 % 1)
({(<scan,1>,6)},12968 % 1)
({(<scan,1>,7)},43019 % 1)

({(<scan,180>,0)},890 % 1)
({(<scan,180>,1)},19205 % 1)
({(<scan,180>,2)},25416 % 1)
({(<scan,180>,3)},23958 % 1)
({(<scan,180>,4)},18528 % 1)
({(<scan,180>,5)},16419 % 1)
({(<scan,180>,6)},14367 % 1)
({(<scan,180>,7)},53518 % 1)

({(motor,0)},17809 % 1)
({(motor,1)},136432 % 1)
({(motor,2)},18060 % 1)

({(location,door12)},2067 % 1)
({(location,door13)},2365 % 1)
({(location,door14)},2012 % 1)
({(location,door45)},1288 % 1)
({(location,door56)},2314 % 1)
({(location,room1)},42708 % 1)
({(location,room2)},19975 % 1)
({(location,room3)},17110 % 1)
({(location,room4)},45058 % 1)
({(location,room5)},16658 % 1)
({(location,room6)},20746 % 1)

({(position,centre)},41677 % 1)
({(position,corner)},38736 % 1)
({(position,side)},91888 % 1)
```
We then *induced* random regional *model* 26,
```
./main induce model026 8 >model026.log

./main entropy_region model011 1 data009
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 1.97725e+06

./main entropy_region model026 1 data009
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 1.96486e+06
```
Its *likelihood* is very similar to that of *model* 11.

Then we *induced model* 27 and compared it to *model* 18,
```
./main induce model027 32 >model027.log

./main entropy model018 1 data009
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 229325

./main entropy model027 1 data009
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 231911

./main observe data008 model018 data009 location
100.0*match_count/z: 59.3599

./main observe data008 model027 data009 location
100.0*match_count/z: 63.9114
```
Both the *likelihood* and the `location` accuracy were a little higher in *model* 27.

Then we *conditioned model* 28 on `location` and compared it to *model* 19,
```
./main condition model028 8 location >model028_location.log

./main entropy model019_location 1 data009
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 220714

./main entropy model028_location 1 data009
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 215919

./main observe data008 model019_location data009 location
100.0*match_count/z: 75.893

./main observe data008 model028_location data009 location
100.0*match_count/z: 86.698
```
*Model* 28 has lower *likelihood* but higher label accuracy than *model* 19 in test dataset 8.


