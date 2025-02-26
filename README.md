# TBOT01 - TurtleBot3 controller

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we develop a TurtleBot3 controlled by unsupervised *induced models*, as well as by partially or wholly supervised *conditioned models*. We demonstrate that it is possible for the TurtleBot3 to learn about its environment without any predefined label or goal, and then to apply this knowledge to accomplish a [task](https://github.com/caiks/TBOT01_ws/blob/master/actor_env010_model028_location_room6_data009_mode002.mp4?raw=true).

The *model* is trained using the *inducers* and *conditioners* implemented in the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC). The AlignmentRepaC repository is a fast C++ implementation of some of the *practicable inducers* described in the paper *The Theory and Practice of Induction by Alignment* at https://greenlake.co.uk/. The *models* are *induced* in the main executable, which can run without the need to install TurtleBot3.

The *history* (training data) is collected by running various controllers in the [turtlebot3 house](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#3-turtlebot3-house) simulated in the [Gazebo](http://gazebosim.org/) virtual environment.

## Sections

[Download, build and run main executable](#main)

[Download, build and run TurtleBot3 nodes](#controller)

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
cat data010a* >data010.bin
cat data011a* >data011.bin
cat data013a* >data013.bin
cat data015a* >data015.bin
cat data016a* >data016.bin

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
The `main` executable has various modes. Note that the `induce` and `condition` *modelling* modes can require up to 24GB of memory depending on the parameterisation,
```
cd ../TBOT01_ws
ln -s ../TBOT01_build/main main

./main stats data001
./main stats data002_room1
./main stats data002_room4
./main stats data003
./main stats data008
./main stats data009

./main bitmap data001
./main bitmap data001 3
./main bitmap data002_room1
./main bitmap data003
./main bitmap data003 10
./main bitmap data003 10 0 59
./main bitmap data005 10 0 59
./main bitmap data008 10 0 59
./main bitmap data009 10 0 59

./main bitmap_average data001
./main bitmap_average data002_room1 20
./main bitmap_average data002_room4 20

./main analyse data002
./main analyse data003
./main analyse data006_01
./main analyse data006_02
./main analyse data006_03
./main analyse data006_04
./main analyse data006_05
./main analyse data007
./main analyse data008
./main analyse data008 substrate003
./main analyse data008 substrate004
./main analyse data009
./main analyse data009 substrate004
./main analyse data009 substrate005
./main analyse data009 substrate006

./main induce model001 4 >model001.log
./main induce model002 4 >model002.log
./main induce model003 4 >model003.log
./main induce model004 4 >model004.log
./main induce model005 4 >model005.log
./main induce model008 4 >model008.log
./main induce model009 4 >model009.log
./main induce model010 4 >model010.log
./main induce model011 4 >model011.log
./main induce model012 4 >model012.log
./main induce model015 4 >model015.log
./main induce model017 8 >model017.log
./main induce model018 8 >model018.log
./main induce model021 16 1 1 >model021_1_1.log
./main induce model021 16 2 4 >model021_2_4.log
./main induce model021 32 3 4 >model021_3_4.log
./main induce model021 8 1 1 >model021_1_1.log
./main induce model021 8 2 4 >model021_2_4.log
./main induce model023 16 1 1 >model023_1_1.log
./main induce model023 16 2 4 >model023_2_4.log
./main induce model023 8 1 1 >model023_1_1.log
./main induce model024 8 7 2  >model024.log
./main induce model026 8 >model026.log
./main induce model027 32 >model027.log

./main condition model006 4 location >model006_location.log
./main condition model006 4 motor >model006_motor.log
./main condition model006 4 position >model006_position.log
./main condition model007 4 location >model007_location.log
./main condition model007 4 motor >model007_motor.log
./main condition model007 4 position >model007_position.log
./main condition model013 4 location >model013_location.log
./main condition model013 4 motor >model013_motor.log
./main condition model013 4 position >model013_position.log
./main condition model014 4 location >model014_location.log
./main condition model014 4 motor >model014_motor.log
./main condition model014 4 position >model014_position.log
./main condition model016 1 motor >model016_motor.log
./main condition model016 4 location >model016_location.log
./main condition model016 4 motor >model016_motor.log
./main condition model016 4 position >model016_position.log
./main condition model016 8 location >model016_location.log
./main condition model016 8 position >model016_position.log
./main condition model019 8 location >model019_location.log
./main condition model019 8 position >model019_position.log
./main condition model020 8 location 1 1 >model020_location_1_1.log
./main condition model020 8 location 2 4 >model020_location_2_4.log
./main condition model020 8 location 3 4 >model020_location_3_4.log
./main condition model020 8 location 7 4 >model020_location_7_4.log
./main condition model022 16 location 1 1 >model022_location_1_1.log
./main condition model022 16 location 2 4 >model022_location_2_4.log
./main condition model022 8 location 1 1 >model022_location_1_1.log
./main condition model025 8 location 7 2 >model025_location_7_2.log
./main condition model028 8 location >model028_location.log
./main condition model029 8 location_next >model029_location_next.log
./main condition model030 8 location_next >model030_location_next.log
./main condition model031 16 location_next >model031_location_next.log
./main condition model032 16 room_next >model032_room_next.log
./main condition model033 16 room_next >model033_room_next.log
./main condition model034 16 room_next >model034_room_next.log

./main bitmap_model model001 
./main bitmap_model model002 
./main bitmap_model model003 
./main bitmap_model model004 
./main bitmap_model model005 
./main bitmap_model model009 5 data003
./main bitmap_model model012 data003

./main entropy model001 10
./main entropy model001 10 data003
./main entropy model001 100
./main entropy model004 10
./main entropy model005 10
./main entropy model006_location 10
./main entropy model006_motor 10
./main entropy model006_position 10
./main entropy model007_location 10
./main entropy model007_motor 10
./main entropy model007_position 10
./main entropy model008 10
./main entropy model009 1 data004
./main entropy model009 10 data003
./main entropy model010 1 data004
./main entropy model010 10 data003
./main entropy model012 1 data004
./main entropy model012 10 data003
./main entropy model013_location 1 data004
./main entropy model013_location 10 data003
./main entropy model013_motor 10 data003
./main entropy model013_position 10 data003
./main entropy model014_location 1 data004
./main entropy model014_location 10 data003
./main entropy model014_location 10 data004
./main entropy model014_motor 10 data003
./main entropy model014_motor 10 data004
./main entropy model014_position 10 data003
./main entropy model014_position 10 data004
./main entropy model016_location 1 data004
./main entropy model016_location 10 data004
./main entropy model016_motor 10 data004
./main entropy model016_position 10 data004
./main entropy model017 1 data004
./main entropy model017 10 data004
./main entropy model018 1 data004
./main entropy model018 1 data009
./main entropy model019_location 1 data004
./main entropy model019_location 1 data009
./main entropy model020_location_1_1 1 data004 1 1
./main entropy model027 1 data009
./main entropy model028_location 1 data009
./main entropy model029_location_next 1 data009 substrate003
./main entropy model030_location_next 1 data009 substrate003
./main entropy model031_location_next 1 data009 substrate003
./main entropy model032_room_next 1 data009 substrate004
./main entropy model033_room_next 1 data009 substrate004
./main entropy model034_room_next 1 data009 substrate004

./main entropy_region model011 1 data004
./main entropy_region model011 1 data004 1
./main entropy_region model011 1 data009
./main entropy_region model015 1 data004
./main entropy_region model015 1 data004 1
./main entropy_region model026 1 data009

./main entropy_region_sequence model024 1 7 2

./main entropy_sequence model020_location_1_1 1 data004 1 1
./main entropy_sequence model020_location_2_4 1 data004 2 4
./main entropy_sequence model020_location_3_4 1 data004 3 4
./main entropy_sequence model020_location_7_4 1 data004 7 4
./main entropy_sequence model021_1_1 1 data004 1 1
./main entropy_sequence model021_2_4 1 data004 2 4
./main entropy_sequence model021_3_4 1 data004 3 4
./main entropy_sequence model022_location_1_1 1 data004 1 1
./main entropy_sequence model022_location_2_4 1 data004 2 4
./main entropy_sequence model023_1_1 1 data004 1 1
./main entropy_sequence model023_2_4 1 data004 2 4
./main entropy_sequence model025_location_7_2 1 data004 7 2

./main entropy_room_next model027 data009
./main entropy_room_next model028_location data009
./main entropy_room_next model029_location_next data009
./main entropy_room_next model030_location_next data009
./main entropy_room_next model031_location_next data009
./main entropy_room_next model032_room_next data009
./main entropy_room_next model033_room_next data009
./main entropy_room_next model034_room_next data009

./main observe data003 model016_location data004 location
./main observe data003 model016_location data004 location 0 59
./main observe data003 model016_position data004 position
./main observe data003 model017 data004 location
./main observe data003 model017 data004 location 0 59
./main observe data003 model017 data004 position
./main observe data004_04 model016_location data004 location
./main observe data004_04 model017 data004 location
./main observe data005 model016_location data004 location
./main observe data005 model016_location data004 location 0 59
./main observe data005 model016_position data004 position
./main observe data005 model017 data004 location
./main observe data005 model017 data004 position
./main observe data005 model018 data004 location
./main observe data005 model018 data004 position
./main observe data005 model019_location data004 location
./main observe data005 model019_position data004 position
./main observe data008 model018 data009 location
./main observe data008 model019_location data009 location
./main observe data008 model027 data009 location
./main observe data008 model028_location data009 location
./main observe data008 model029_location_next data009 location 0 0 substrate003
./main observe data008 model029_location_next data009 location_next 0 0 substrate003
./main observe data008 model030_location_next data009 location 0 0 substrate003
./main observe data008 model030_location_next data009 location_next 0 0 substrate003
./main observe data008 model031_location_next data009 location 0 0 substrate003
./main observe data008 model031_location_next data009 location_next 0 0 substrate003
./main observe data008 model032_room_next data009 location 0 0 substrate004
./main observe data008 model032_room_next data009 location_next 0 0 substrate004
./main observe data008 model032_room_next data009 room_next 0 0 substrate004
./main observe data008 model033_room_next data009 location 0 0 substrate004
./main observe data008 model033_room_next data009 location_next 0 0 substrate004
./main observe data008 model033_room_next data009 room_next 0 0 substrate004
./main observe data008 model034_room_next data009 location 0 0 substrate004
./main observe data008 model034_room_next data009 location_next 0 0 substrate004
./main observe data008 model034_room_next data009 room_next 0 0 substrate004

./main observe_bitmap data003 model009 data003 10 0 59
./main observe_bitmap data003 model010 data003 10 0 59
./main observe_bitmap data003 model012 data003 10 0 59
./main observe_bitmap data003 model013_location data003 10 0 59
./main observe_bitmap data003 model014_location data003 10 0 59
./main observe_bitmap data003 model016_location data004 10 0 59
./main observe_bitmap data003 model017 data004 10 0 59
./main observe_bitmap data005 model016_location data004 10 0 59
./main observe_bitmap data005 model017 data004 10 0 59
./main observe_bitmap data005 model018 data004 10 0 59
./main observe_bitmap data005 model019_location data004 10 0 59

./main observe_sequence data005 model020_location_1_1 data004 location 1 1
./main observe_sequence data005 model020_location_2_4 data004 location 2 4
./main observe_sequence data005 model020_location_3_4 data004 location 3 4
./main observe_sequence data005 model020_location_7_4 data004 location 7 4
./main observe_sequence data005 model021_1_1 data004 location 1 1
./main observe_sequence data005 model021_2_4 data004 location 2 4
./main observe_sequence data005 model021_3_4 data004 location 3 4
./main observe_sequence data005 model022_location_1_1 data004 location 1 1
./main observe_sequence data005 model022_location_2_4 data004 location 2 4
./main observe_sequence data005 model023_1_1 data004 location 1 1
./main observe_sequence data005 model023_2_4 data004 location 2 4
./main observe_sequence data005 model025_location_7_2 data004 location 7 2

./main room_expected data003 room1
./main room_expected data004 room1
./main room_expected data008 room1
./main room_expected data008 room5
./main room_expected data009 room1
./main room_expected data009 room5
./main room_expected data009 room5 17 data010
./main room_expected data009 room5 17 data011
./main room_expected data009 room5 17 data012
./main room_expected data009 room5 17 data019
./main room_expected data010 room5 17 data020
./main room_expected data010 room5
./main room_expected data010 room5 17 data015
./main room_expected data011 room5
./main room_expected data011 room5 17 data016
./main room_expected data019 room5 17 data018

```
<a name = "controller"></a>

## Download, build and run TurtleBot3 nodes

To run the turtlebot it is necessary to install [ROS2](https://index.ros.org/doc/ros2/), [Gazebo](http://gazebosim.org/tutorials?cat=install) and [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_simulation/#simulation) on a machine with a GPU and at least 4GB of memory.

[AWS EC2 instance](#AWS)

[Windows 10 WSL2 instance](#Windows)

[Installation](#Installation)

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

```

```
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

cd ~/turtlebot3_ws/src/TBOT01_ws
cat data008a* >data008.bin
cat data009a* >data009.bin
cat data010a* >data010.bin
cat data011a* >data011.bin
cat data013a* >data013.bin
cat data015a* >data015.bin
cat data016a* >data016.bin

cd ~/turtlebot3_ws/src
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

Now let us investigate various turtlebot *models* and controllers. 

[Sensors, motors, environment and the collision avoidance controller](#Sensors)

[Models of the scan substrate](#Models)

[Models conditioned on location and position](#Models_conditioned)

[Location and position observer](#Observer)

[Modelling with an unbiased controller](#Unbiased)

[Timewise frames](#Timewise)

[Motor actions](#Motor)

[Actor node](#Actor)

<a name = "Sensors"></a>

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

You can view a short video [here](https://github.com/caiks/TBOT01_ws/blob/master/data002_room1.mp4?raw=true). In this simulation turtlebot leaves room 1 and goes into room 2.

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

<a name = "Models"></a>

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

<a name = "Models_conditioned"></a>

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

<a name = "Observer"></a>

### Location and position observer

Now let us use the *models* we have created to make guesses about the `location` and `position` in a ROS node that observes the turtlebot at it moves around the turtlebot house in the gazebo simulation. The `TBOT01` [observer](https://github.com/caiks/TBOT01/blob/master/observer.h) node is given a *model*, a label *variable* and a observe interval. At each observation it *applies* the *model* to the current *event* to determine its *slice*. The prediction of the label is the most common *value* of the label *variable* in the `data002` *history's slice*. The prediction is reported along with the actual *value*, calculated from the current *event's* odometry, and a running average of the matches is calculated.

Let us consider *models* `model006_location` and `model006_position` which are *conditioned* on the label given the `scan` *substrate*. The following are run in separate shells,
```
gazebo -u ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data.bin 250

```
```
ros2 run TBOT01 observer model006_location location 2500

```
```
ros2 run TBOT01 observer model006_position position 2500

```
The turtlebot is allowed to run around for a while.

This is the output for `location`,
```
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

<a name = "Unbiased"></a>

### Modelling with an unbiased controller

Now let us see if we can encourage the turtlebot to travel between rooms by removing the bias to the right. We will set an interval that alternates the bias.

The simulation was restarted in room 4,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
and the controller re-run, alternating the bias every 1000 ms,
```
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
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```
and the controller re-run, alternating the bias every 1000 ms,
```
ros2 run TBOT01 controller data004_01.bin 250 1000

```
```
ros2 run TBOT01 observer model013_location location 2500

```
```
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
ros2 run TBOT01 controller data004_02.bin 250 1000

```
```
ros2 run TBOT01 observer model014_location location 2500 data003
...
room1    room4   fail    63.630184
```
```
ros2 run TBOT01 observer model014_position position 2500 data003
...
centre   centre  match   79.918312
```
The *model* 14 `location` accuracy is a little higher at around 64% and the `position` accuracy is higher at around 80%.

We can compare the *conditioned models*, 13 and 14, above to the three *induced models* 9, 10 and 12,

```
ros2 run TBOT01 controller data004_03.bin 250 1000

```
```
ros2 run TBOT01 observer model009 location 2500 data003
...
room4    room1   fail    39.401294
```
```
ros2 run TBOT01 observer model009 position 2500 data003
...
centre   centre  match   59.902991
```
The *model* 9 `location` accuracy is a considerably lower than either of the *conditioned models* at around 39% and the `position` accuracy is also lower at around 60%.

```
ros2 run TBOT01 controller data004_04.bin 250 1000

```
```
ros2 run TBOT01 observer model010 location 2500 data003
...
room2    room1   fail    41.637990

```
```
ros2 run TBOT01 observer model010 position 2500 data003
...
corner   corner  match   56.129477
```
The *model* 10 is the same as *model* 9 but with a greater `fmax`. Its `location` accuracy is a slightly higher at around 42% and the `position` accuracy is  lower at around 56%.
```
ros2 run TBOT01 controller data004_05.bin 250 1000

```
```
ros2 run TBOT01 observer model012 location 2500 data003
...
room4    room5   fail    42.396313
```
```
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
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data.bin 250 1000

```
```
ros2 run TBOT01 observer model016_location location 2500 data004
...
room6    room6   match   85.122898
```
```
ros2 run TBOT01 observer model016_position position 2500 data004
...
side     side    match   88.745149
```
Now let us *induce* a *model* to a similar depth for comparison. *Model* 17 has the same parameters as *model* 12, but the *underlying model* is *model* 15 and the `fmax` is increased to 4096,
```
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
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env002.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data.bin 250 1000

```
```
ros2 run TBOT01 observer model017 location 2500 data004
...
door56   room5   fail    58.469388
```
```
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

<a name = "Timewise"></a>

### Timewise frames

*Model* 16 was *conditioned* on 81261 *events* and has 28315 *transforms*. No doubt larger *models* *conditioned* on more *history* would incrementally increase the label accuracy, but for now let us consider increasing the *substrate* instead with timewise *frames*.

To do that we must first select which of the past records will comprise the short term memory, i.e. the timewise *frames*. Let us image the first 60 *events* in `data003`,
```
./main bitmap data003 10 0 59

```
Each *event* is scaled vertically by 10 pixels,

![data003](images/data003.bmp?raw=true)

In these 12 seconds, the turtlebot starts in the room 4, facing to the right, turns to the left by 30 degrees and then moves towards the corridor to room 1,

1 second|12 seconds
---|---
![env002_1s](images/env002_1s.jpg?raw=true)|![env002_12s](images/env002_12s.jpg?raw=true)

We can *apply* a *model* to the 60 *events* and average the *slice* corresponding to each *event* to see the *history* as turtlebot 'sees' it. For example, if we *apply* *model* 9 with the *model* *history* `data003`,
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

We can see that the addition of the *values* as they were in the *events* a few seconds ago makes little difference to the *likelihood* and in all cases reduces the `location` accuracy. Clearly the *alignments* within the *frames* are much greater than those between the *frames*, suggesting that for the `data004` *history size*, at least, the static information is more important than the dynamic. Also the dynamic information perhaps causes some over-fitting in the *conditioned models* leading to lower *likelihoods* and label accuracies.

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

*Model* 25 is a three *level model* with the top *level conditioned* on a spacewise *substrate* of two *level model 24 induced* on a timewise *substrate* of one *level model 11 induced* on a random region *substrate*. Compare it to *model* 22 which is a three *level model* with the top *level conditioned* on a timewise *substrate* of two *level model 18 induced* on a spacewise *substrate* of one *level model 11 induced* on a random region *substrate*. *Model* 25 has a lower *likelihood* but a higher accuracy than *model* 22. So *model* 25 is intermediate between *model* 22 and *model* 20, which is the two *level conditioned* on a spacetimewise *substrate* of one *level model 11 induced* on a random region *substrate*. This confirms that the static information is more important than the dynamic. We will ignore timewise *models* for the remainder of this discussion.

<a name = "Motor"></a>

### Motor actions

Now that TurtleBot is able to make guesses about its location, let us consider what actions TurtleBot should take in order to navigate autonomously around the TurtleBot house.

In order to see the consequences of past actions, we must look at the sequence of *events*. In this way we can determine future labels for each *event*, for example, `location_next` and `room_next`. It was found, however, that the existing collision avoidance algorithm was not enough to prevent occasional crashes in the previous datasets, making it difficult to obtain long sequences. The crashes were due to collisions with the open shelves and bookcases. The solution was to simply turn the shelves and bookcases around so that they were closed and thus detectable by the turtlebot's lidar. The new environment is `env009`.

In addition, we have added a random turn interval to the controller. If the turtlebot is going straight it will randomly turn left or right on a timescale on the order of the turn interval.

We ran the simulation with the new environment,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```
and ran the new controller with a random turn interval of 5000 ms,
```
ros2 run TBOT01 controller data008.bin 250 5000 5000 

```
Note that the bias interval has been increased to 5000 ms to reduce dither in the corners.

The turtlebot ran for 5.5 hours to create a test dataset `data008`,
```
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
In `data008` 3% more time is spent in a turn but 2% less time is spent in a corner.

We ran the turtlebot again to create a training dataset, `data009`, this time for 12 hours -
```
ros2 run TBOT01 controller data009.bin 250 5000 5000 

```
Now the turtlebot spends 48% of its time in rooms 4, 5 and 6 -
```
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
Both the *likelihood* and the `location` accuracy were a little higher in *model* 27 using the new dataset.

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

Now let us create new *substrate variables* `location_next` and `position_next`. These are *variables* that look forward from the current *event* until a `location` or `position` transition. A *history* in the new *substrate* 3 is obtained with the  `recordListsHistoryRepa_3` function in [dev.h](https://github.com/caiks/TBOT01/blob/master/dev.h),
```
./main analyse data008 substrate003
...
({(location_next,door12)},7951 % 1)
({(location_next,door13)},8184 % 1)
({(location_next,door14)},16717 % 1)
({(location_next,door45)},10830 % 1)
({(location_next,door56)},18869 % 1)
({(location_next,room1)},1073 % 1)
({(location_next,room2)},298 % 1)
({(location_next,room3)},258 % 1)
({(location_next,room4)},3528 % 1)
({(location_next,room5)},9808 % 1)
({(location_next,room6)},998 % 1)
({(location_next,unknown)},98 % 1)

({(position_next,centre)},22150 % 1)
({(position_next,corner)},20604 % 1)
({(position_next,side)},35845 % 1)
({(position_next,unknown)},13 % 1)
```
Now let us *condition model* 29 on `location_next` rather than `location`,
```
./main condition model029 8 location_next >model029_location_next.log
...
fud: 4096
fud slice size: 39
sized entropy label : 58.8561
...

./main entropy model029_location_next 1 data009 substrate003
...
fudRepasSize(*dr->fud): 18712
frder(*dr->fud)->size(): 4097
frund(*dr->fud)->size(): 360
treesSize(*dr->slices): 8192
treesLeafElements(*dr->slices)->size(): 4097
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 213890

./main observe data008 model029_location_next data009 location 0 0 substrate003
...
100.0*match_count/z: 82.0422

./main observe data008 model029_location_next data009 location_next 0 0 substrate003
...
100.0*match_count/z: 54.0732
```
The *likelihood* of 213,890 is a little less than *model* 28. The label accuracy is a lot lower at 54% instead of 87%, but of course the label is considerably more ambiguous being the next `location` rather than the current `location`.

Now let us add `motor` to the *substrate* accessible to the *conditioner*, to see if it can disambiguate the next label.
```
./main condition model030 8 location_next >model030_location_next.log
...
fud: 4096
fud slice size: 61
sized entropy label : 57.0735
...

./main entropy model030_location_next 1 data009 substrate003
...
fudRepasSize(*dr->fud): 18813
frder(*dr->fud)->size(): 4240
frund(*dr->fud)->size(): 361
treesSize(*dr->slices): 8335
treesLeafElements(*dr->slices)->size(): 4240
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 213915

./main observe data008 model030_location_next data009 location 0 0 substrate003
...
100.0*match_count/z: 81.7852

./main observe data008 model030_location_next data009 location_next 0 0 substrate003
...
100.0*match_count/z: 53.9421
```
We can see that the *likelihood* and label accuracy are very similar to *model* 29. However the *underlying* has 361 *variables* which means that it includes `motor`. When we examine the log we can see that `motor` is the *entropy variable* in 143 of the 4096 *fuds* and that the trailing *sized slice entropy* is 57.0735 rather than 58.8561 without `motor` (compare to 18.1327 for `location` *conditioned model* 28). That is, `motor` is having some effect in *conditioning* the *model* even if it does not show itself in the accuracy.

In order to increase the effect let us use an `fmax` of 16,384 in *model* 31,
```
./main condition model031 16 location_next >model031_location_next.log

./main entropy model031_location_next 1 data009 substrate003
...
fudRepasSize(*dr->fud): 49897
frder(*dr->fud)->size(): 16909
frund(*dr->fud)->size(): 361
treesSize(*dr->slices): 33292
treesLeafElements(*dr->slices)->size(): 16909
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 218519

./main observe data008 model031_location_next data009 location_next 0 0 substrate003
...
100.0*match_count/z: 56.0576
```
Now there is a small increase in both the *likelihood*, at 218,519, and the next label accuracy, at 56%. In total, 524 of the 16,384 *fuds depend* on `motor`,

Model|Type|Underlying|Fmax|Dataset|Substrate|Likelihood|Location %|Next Location %
---|---|---|---|---|---|---|---|---
18|induced|model 11|4096|4|2|229,325|59|
27|induced|model 26|4096|9|2|231,911|64|
19|conditioned|model 11|4096|4|2|220,714|76|
28|conditioned|model 26|4096|9|2|215,919|87|
29|conditioned|model 26|4096|9|3|213,890|82|54
30|conditioned|model 26|4096|9|3|213,915|82|54
31|conditioned|model 26|16384|9|3|218,519|86|56

The `motor` *value* has at least a weak functional relation to the next `location`, so let us see what effect there is if we restrict the `location` *values* to rooms only. Here is `substrate004` which adds `room_next`,

```
/main analyse data008 substrate004
...
({(room_next,door12)},0 % 1)
({(room_next,door13)},0 % 1)
({(room_next,door14)},0 % 1)
({(room_next,door45)},0 % 1)
({(room_next,door56)},0 % 1)
({(room_next,room1)},19135 % 1)
({(room_next,room2)},2013 % 1)
({(room_next,room3)},5331 % 1)
({(room_next,room4)},14994 % 1)
({(room_next,room5)},31626 % 1)
({(room_next,room6)},5415 % 1)
({(room_next,unknown)},98 % 1)
```
Now we run *models* 32, 33 and 34 which parallel *models* 29, 30 and 31 but are *conditioned* on `room_next` rather than `location_next`,
```
./main condition model032 16 room_next >model032_room_next.log
...
fud: 4096
fud slice size: 78
sized entropy label : 48.3379
...

./main entropy model032_room_next 1 data009 substrate004
...
fudRepasSize(*dr->fud): 19603
frder(*dr->fud)->size(): 4097
frund(*dr->fud)->size(): 360
treesSize(*dr->slices): 8192
treesLeafElements(*dr->slices)->size(): 4097
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 212715

./main observe data008 model032_room_next data009 location 0 0 substrate004
...
100.0*match_count/z: 80.9647

./main observe data008 model032_room_next data009 location_next 0 0 substrate004
...
100.0*match_count/z: 51.454

./main observe data008 model032_room_next data009 room_next 0 0 substrate004
...
100.0*match_count/z: 63.3211

./main condition model033 16 room_next >model033_room_next.log
...
fud: 4096
fud slice size: 340
sized entropy label : 46.8731
...

./main entropy model033_room_next 1 data009 substrate004
...
fudRepasSize(*dr->fud): 19309
frder(*dr->fud)->size(): 4216
frund(*dr->fud)->size(): 361
treesSize(*dr->slices): 8311
treesLeafElements(*dr->slices)->size(): 4216
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 212496

./main observe data008 model033_room_next data009 location 0 0 substrate004
...
100.0*match_count/z: 80.9533

./main observe data008 model033_room_next data009 location_next 0 0 substrate004
...
100.0*match_count/z: 51.5354

./main observe data008 model033_room_next data009 room_next 0 0 substrate004
...
100.0*match_count/z: 63.1672

./main condition model034 16 room_next >model034_room_next.log

./main entropy model034_room_next 1 data009 substrate004
...
fudRepasSize(*dr->fud): 50137
frder(*dr->fud)->size(): 16878
frund(*dr->fud)->size(): 361
treesSize(*dr->slices): 33261
treesLeafElements(*dr->slices)->size(): 16878
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 215763

./main observe data008 model034_room_next data009 location 0 0 substrate004
...
100.0*match_count/z: 83.9261

./main observe data008 model034_room_next data009 location_next 0 0 substrate004
...
100.0*match_count/z: 52.6523

./main observe data008 model034_room_next data009 room_next 0 0 substrate004
...
100.0*match_count/z: 64.1289
```
We can see that the *likelihood* and label accuracy of *model* 33 are very similar to *model* 32, but we can see that `motor` is the *entropy variable* in 119 of the 4096 *fuds* and that the trailing *sized slice entropy* is 46.8731 rather than 48.3379. 

In *model* 34 there is an increase in both the *likelihood* and the next label accuracy. In total, 493 of the 16,384 *fuds depend* on `motor`,

Model|Type|Underlying|Fmax|Dataset|Substrate|Likelihood|Location %|Next Location %|Next Room %
---|---|---|---|---|---|---|---|---|---
18|induced|model 11|4096|4|2|229,325|59||
27|induced|model 26|4096|9|2|231,911|64||
19|conditioned|model 11|4096|4|2|220,714|76||
28|conditioned|model 26|4096|9|2|215,919|87||
29|conditioned|model 26|4096|9|3|213,890|82|54|
30|conditioned|model 26|4096|9|3|213,915|82|54|
31|conditioned|model 26|16384|9|3|218,519|86|56|
32|conditioned|model 26|4096|9|4|212,715|81|51|63
33|conditioned|model 26|4096|9|4|212,496|81|52|63
34|conditioned|model 26|16384|9|4|215,763|84|53|64

<a name = "Actor"></a>

### Actor node

Given that the `motor` *value* has at least a weak functional relation to the `location_next` and `room_next`, we modified the controller to accept turn requests made by an observer or by a user. These requests are only accepted when the turtlebot is moving straight ahead. If the turtlebot is avoiding an obstacle the turn request merely sets the bias. Let us now consider how to issue these turn requests.

The `TBOT01` [actor](https://github.com/caiks/TBOT01/blob/master/actor.h) node is similar to the observer node. It is given a *model*, a goal room and a mode of deciding actions. At each potential action it *applies* the *model* to the current *event* to determine its *slice*. The *slice* of the given *history*, e.g. `data009`, is *reduced* to a *histogram* of the label *variables* `location`, `motor` and `room_next`. 

In the simplest mode, `mode001`, this label *histogram* is *multiplied* by a *unit histogram* that defines the desired `room_next` given the goal room and the *slice's* `location`. For example, if the goal is room 6 and the `location` is room 1 then the `room_next` is room 4, rather than rooms 2 or 3. The turtlebot guesses `location` and then repeats the `motor` actions that tended in the past to lead to the desired goal. That is, the requested action is chosen at random according to the *probability histogram* implied by the *normalised reduction* to `motor`.

In order to test whether `TBOT01` actor is navigating around the turtlebot house better than chance, its goal will be set from a fixed sequence of randomly selected rooms. As soon as turtlebot has reached the current goal room, the next goal is set from the next room in the infinite sequence. We will let the turtlebot run until we obtain a degree of statistical significance for the average journey time as measured by the standard score. First we obtain the distribution for the random-turn turtlebot by calculating the journey times in `data009`,
```
./main room_expected data009 room5
...
dataset: data009
z: 172301
n: 55
counts: [141,10209,4340,1196,8987,1176,5103,712,2532,1887,379,1218,204,69,2584,1079,2296,15562,6697,443,1376,3717,7213,66,2610,852,675,984,1013,3090,2015,73,1242,3410,4143,3905,3806,2174,2906,496,8126,3280,4505,163,10837,3848,3272,1527,11867,619,115,218,7594,3393,180]
mean: 3129.53
sqrt(variance): 3368.92
sqrt(variance/n): 454.265
```
The turtlebot was started in room 4 with a first goal of room 5. It travels to 55 rooms during the 12 hours of wandering around with random turns every 5 seconds or so. The average journey time is 3129.53 *events* at 4 fps, around 13 minutes. The standard error is 454.265 *events* or around 2 minutes.

To set the turtlebot's goal rooms in the same sequence we use the `TBOT01` [commander](https://github.com/caiks/TBOT01/blob/master/commander.h) node. This publishes the goals to which the actor node subscribes. The first test is with the actor in `mode001` using *conditioned model* 28,

```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data012.bin 250 5000

```
```
ros2 run TBOT01 actor model028_location room5 1000 data009 0 mode001

```

```
ros2 run TBOT01 commander room5 250

```
This shows the statistics of the journeys taken -
```
./main room_expected data009 room5 17 data012
...
dataset2: data012
z: 196722
n: 75
counts: [83,1838,1237,4332,1570,1041,1050,1294,16844,497,224,5599,7208,441,128,67,2955,2757,2083,222,1207,924,3175,1174,6362,4783,1288,2476,729,8118,2487,51,5613,2952,427,2292,771,1506,4346,458,11283,1096,5298,187,2115,1386,1067,2993,797,587,2700,109,1009,3067,3097,198,4091,172,91,6245,1773,982,1629,1547,4494,3399,863,1970,433,1059,12756,3682,459,1087,1413]
mean: 2503.24
sqrt(variance): 2960.09
sqrt(variance/n): 341.802
sqrt(err1*err1 + err2*err2): 568.494
(mean2-mean1)/sqrt(err1*err1 + err2*err2): -1.10166
```
After a run of 13 hours, the standard score is not yet significant, but it is better than one standard deviation. The average journey time of the mode 1 turtlebot is now around ten and a half minutes, though with a standard error of over one and a half minutes.

It is probable that the mode 1 turtlebot shows some improvement over chance. It works because it selects the behaviour of the random-turn turtlebot which led in the past to the correct next room given the goal. Observing the behaviour of the mode 1 turtlebot it appears that it works reasonably well when it is near the desired exit from the room. In these cases there is enough *history* so that the correct action, left, right or straight ahead, is considerably more probable than the incorrect actions. When the turtlebot is far from the correct exit, however, the probabilities are not much different from chance and then the mode 1 turtlebot behaves no better than the random-turn turtlebot. In mode 2 we try to address this issue.

Another issue concerns rooms with only one exit, i.e. rooms 2, 3 and 6. In these cases all actions lead to the correct next room, so the turtlebot always behaves like the random-turn turtlebot. The random turns slightly increase the time spent in the room compared to the simple collision avoidance turtlebot, so lengthening journey times. This can be seen in the analysis of `data009` and `data003` above.

In mode 2 the turtlebot *subtracts* the correct `room_next` label *histogram* from the *slice's* label *histogram* to obtain the remaining incorrect *histogram* of the `motor` action, i.e. actions that led to the wrong rooms given the goal. It then adds the *count* of right turns of the incorrect *histogram* to the *count* of left turns of the correct *histogram* and vice-versa. The mode 2 turtlebot is thus attracted to the correct exit and repelled from incorrect exits. In addition, in mode 2 the turtlebot disables the turns when it thinks it is in the single exit rooms, 2, 3 and 6. When we re-run we see an improvement,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data019.bin 250 5000

```
```
ros2 run TBOT01 actor model028_location room5 1000 data009 0 mode002

```

```
ros2 run TBOT01 commander room5 250

```

```
./main room_expected data009 room5 17 data019
...
dataset2: data019
z: 120305
n: 60
counts: [480,1690,4563,1666,930,13587,1299,2698,2560,877,1336,2090,3075,65,238,1109,924,4204,1320,246,2548,7330,4225,159,1261,1294,72,346,1968,1392,3849,79,2489,1900,41,1329,1154,1278,5447,171,5406,416,840,594,244,2102,3658,513,8933,173,1760,160,345,333,174,5623,2357,227,110,2627]
mean: 1998.07
sqrt(variance): 2409.78
sqrt(variance/n): 311.101
sqrt(err1*err1 + err2*err2): 550.582
(mean2-mean1)/sqrt(err1*err1 + err2*err2): -2.05503
```
Now the mean journey time is significantly different. The average journey time has decreased from 13 minutes to 8  minutes and 20 seconds plus or minus 1 minute and 17 seconds.

Further progress with this mode could perhaps be made by recursing on the *history*. The dataset of a mode 2 turtlebot's run would become the dataset given, along with the *model*, to a re-run of the mode 2 turtlebot. That is, the attraction/repulsion mode 2 behaviour would replace the random-turn turtlebot's behaviour, so increasing the differences in the *counts* between correct and incorrect actions. This could be repeated several times to increase the differences in probabilities. (Note that this is not reinforcement learning, because no *modelling* is done between runs.)

An alternative approach is to consider a discrete choice function of the differences between correct and incorrect actions, rather than a probabilistic choice function. That is, if the difference between turns is greater than a certain threshold then always choose the turn with the larger *count*, otherwise carry on straight ahead. A reasonable threshold is the fraction of random turns per record, i.e. `250 ms / 5000 ms = 0.05`. A mode 3 turtlebot subtracts the left turn *count* of the incorrect *histogram* from the left turn *count* of the correct *histogram*, and the same for the right turn *count*. (Note that the result can be negative, so is no longer a *histogram count*.) If the difference between the left turn and the right turn is greater than the threshold then the larger turn is always selected. In other words, in the mode 3 turtlebot the action is always determined for each *slice* for the given dataset and goal room. Mode 3 retains the do-nothing behaviour of mode 2 when in single exit rooms.
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data010.bin 250 5000

```
```
ros2 run TBOT01 actor model028_location room5 1000 data009 0 mode003 0.05

```

```
ros2 run TBOT01 commander room5 250

```
```
./main room_expected data009 room5 17 data010
...
dataset2: data010
z: 45175
n: 43
counts: [1287,447,1387,2157,2621,2386,1876,1084,812,644,708,515,2033,284,191,64,411,673,2138,595,1253,435,2811,163,1499,3268,69,136,913,237,636,76,876,1442,478,758,1007,172,1630,144,533,926,3358]
mean: 1049.6
sqrt(variance): 885.097
sqrt(variance/n): 134.976
sqrt(err1*err1 + err2*err2): 473.894
(mean2-mean1)/sqrt(err1*err1 + err2*err2): -4.389
```
This run has a highly significant standard score of -4.39. The *model* 28 mode 3 turtlebot's average journey time is now 4 minutes 22 seconds, plus or minus 34 seconds.

You can view a video of one of mode 3 turtlebot's journeys [here](https://github.com/caiks/TBOT01_ws/blob/master/actor_env010_model028_location_room6_data009_mode002.mp4?raw=true). In this journey turtlebot begins in room 1 with a goal of room 6,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env010.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data.bin 250 5000

```
```
ros2 run TBOT01 actor model028_location room6 1000 data009 0 mode003 0.05

```
Note that this example is unusually fast for a 3 room journey at only 1 minute and 4 seconds. The turtlebot more typically tends to travel in the general direction of the correct exit but often fails to pass through it on the first attempt.

Now that we are sure of a definite improvement in the turtlebot's ability to navigate around the turtlebot house, let us compare the performance  of the partly *conditioned model* 28 and the purely *induced model* 27,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data011.bin 250 5000

```
```
ros2 run TBOT01 actor model027 room5 1000 data009 5 mode003 0.05

```

```
ros2 run TBOT01 commander room5 250

```
```
./main room_expected data009 room5 17 data011
...
dataset2: data011
z: 42400
n: 33
counts: [211,558,1434,843,1469,4057,1193,3029,1015,987,2413,823,398,384,648,290,172,3976,1953,212,5581,496,1606,103,2611,1226,66,888,895,878,1127,21,559]
mean: 1276.42
sqrt(variance): 1277.26
sqrt(variance/n): 222.342
sqrt(err1*err1 + err2*err2): 505.76
(mean2-mean1)/sqrt(err1*err1 + err2*err2): -3.664
```
The *model* 27 mode 3 turtlebot has an average journey time of 5 minutes and 19 seconds, plus or minus 56 seconds. Although the lower `location` accuracy of *model* 27 tends to increase the journey time, we can still see that a completely unsupervised *model* - obtained without requiring any label - can produce purposeful behaviour in the case where there are *alignments* between the sensor *variables* and the motor and goal *variables*.

Modes 2 and 3 simply ignore the single exit rooms. Let us create a new *substrate* `substrate005` which adds to the `location` *values*  extra 'rooms' `room2z`, `room3z` and `room6z`. These form the last 1.5 m of each single exit room's dead end. Rooms `room2`, `room3` and `room6` are correspondingly truncated, 
```
./main analyse data009 substrate005
({(location,door12)},2067 % 1)
({(location,door13)},2365 % 1)
({(location,door14)},2012 % 1)
({(location,door45)},1288 % 1)
({(location,door56)},2314 % 1)
({(location,room1)},42708 % 1)
({(location,room2)},13102 % 1)
({(location,room2z)},6873 % 1)
({(location,room3)},10362 % 1)
({(location,room3z)},6748 % 1)
({(location,room4)},45058 % 1)
({(location,room5)},16658 % 1)
({(location,room6)},13108 % 1)
({(location,room6z)},7638 % 1)
```
The *substrate* `substrate006` adds the redefined `room_next` *variable*,
```
./main analyse data009 substrate006
...
({(room_next,room1)},25554 % 1)
({(room_next,room2)},16927 % 1)
({(room_next,room2z)},10143 % 1)
({(room_next,room3)},26493 % 1)
({(room_next,room3z)},6400 % 1)
({(room_next,room4)},27741 % 1)
({(room_next,room5)},34850 % 1)
({(room_next,room6)},14462 % 1)
({(room_next,room6z)},9659 % 1)
({(room_next,unknown)},72 % 1)
```
Now when the turtlebot is in room 2, say, it can choose between a correct turn to `room1` or an incorrect turn to `room2z`. Note that there is no requirement to change the *model*, merely the definitions of the label *variables* upon which the actions are decided.

Mode 4 is the same as the probabilistic mode 2, but with the *substrate* 6 and no special single exit handling,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```
```
ros2 run TBOT01 controller data018.bin 250 5000

```
```
ros2 run TBOT01 actor model028_location room5 1000 data009 0 mode004

```
```
ros2 run TBOT01 commander room5 250

```
```
./main room_expected data019 room5 17 data018
...
dataset: data019
z: 120305
n: 60
counts: [480,1690,4563,1666,930,13587,1299,2698,2560,877,1336,2090,3075,65,238,1109,924,4204,1320,246,2548,7330,4225,159,1261,1294,72,346,1968,1392,3849,79,2489,1900,41,1329,1154,1278,5447,171,5406,416,840,594,244,2102,3658,513,8933,173,1760,160,345,333,174,5623,2357,227,110,2627]
mean: 1998.07
sqrt(variance): 2409.78
sqrt(variance/n): 311.101
dataset2: data018
z: 113517
n: 53
counts: [365,986,3093,8565,1334,6748,2880,2004,3399,2756,805,3445,860,209,123,663,4583,8095,1021,801,2525,7264,1119,959,1417,684,165,3865,1626,644,374,304,530,2750,202,773,1120,113,1292,63,1598,643,1212,741,2697,7060,2590,3054,4627,375,145,1353,2445]
mean: 2057.81
sqrt(variance): 2133.86
sqrt(variance/n): 293.108
sqrt(err1*err1 + err2*err2): 427.43
(mean2-mean1)/sqrt(err1*err1 + err2*err2): 0.139777
```
In the case of the probabilistic choice mode 4, the new *substrate* appears to have decreased the performance slightly, but not significantly. 

Mode 5 is the same as threshold mode 3, but with the *substrate* 6 and no special single exit handling,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data015.bin 250 5000

```
```
ros2 run TBOT01 actor model028_location room5 1000 data009 0 mode005 0.05 

```

```
ros2 run TBOT01 commander room5 250

```
```
./main room_expected data010 room5 17 data015
...
dataset: data010
room_initial: room5
room_seed: 17
dataset2: data015
dataset: data010
z: 45175
n: 43
counts: [1287,447,1387,2157,2621,2386,1876,1084,812,644,708,515,2033,284,191,64,411,673,2138,595,1253,435,2811,163,1499,3268,69,136,913,237,636,76,876,1442,478,758,1007,172,1630,144,533,926,3358]
mean: 1049.6
sqrt(variance): 885.097
sqrt(variance/n): 134.976
dataset2: data015
z: 57660
n: 72
counts: [231,691,1199,790,2612,1082,934,252,1550,440,495,1067,695,179,496,69,574,405,1947,538,405,630,1062,162,416,2730,77,966,766,331,322,85,523,409,152,356,1612,420,2774,104,543,1821,2386,514,270,1003,1962,1371,577,360,1130,261,545,1377,175,339,503,1049,108,624,1176,304,1299,566,1049,800,326,607,506,1272,446,1186]
mean: 791.708
sqrt(variance): 638.188
sqrt(variance/n): 75.2112
sqrt(err1*err1 + err2*err2): 154.516
(mean2-mean1)/sqrt(err1*err1 + err2*err2): -1.66906
```
This run is significantly different from the mode 3 run with *model* 28. The average journey time has decreased to 3 minutes and 18 seconds, plus or minus 19 seconds, which is the quickest of all of these tests of the actor node. Comparison of the mode 3 dataset `data010` and the mode 5 dataset `data015` shows that the time spent in the single exit rooms has reduced by 4%. Note that this is the case in spite of the fact that *model* 28 was *conditioned* on the old definition of the `location` label. The improvement is less significant over a run of 14 hours,
```
./main room_expected data010 room5 17 data020
...
dataset2: data020
z: 211517
n: 242
counts: [610,898,1401,1320,1802,1032,888,1110,1116,242,958,312,266,68,1827,82,53,2114,774,616,398,1444,1264,59,3662,2789,168,1003,965,650,1762,346,382,594,76,606,399,662,970,600,399,826,951,700,970,2234,729,1489,905,176,267,78,272,220,978,1643,785,350,911,1190,1577,582,1224,634,960,4916,285,692,508,1214,1256,525,231,2242,426,665,2282,424,75,331,704,123,1333,484,1886,348,565,146,2759,1683,82,531,139,1489,607,510,1309,317,883,544,706,2538,765,1514,890,72,1,113,834,1384,206,559,948,724,866,758,1989,97,659,1935,541,335,527,513,436,161,1137,1582,597,288,2696,935,1618,1417,224,572,1204,669,2809,325,142,189,530,892,238,327,765,178,222,32,421,446,220,1445,93,1696,1855,108,820,378,818,684,702,38,445,612,165,2895,648,855,562,1473,69,709,425,467,254,285,573,3855,764,835,329,2408,224,319,395,844,1107,971,1122,2602,661,521,624,1501,1245,1473,137,866,480,3756,2573,433,980,398,72,528,759,750,703,1024,402,448,1573,2161,93,511,244,688,756,412,790,291,837,467,852,1108,736,165,213,1083,94,1708,747,484,851,2495,2621,140,3150,345]
mean: 873.537
sqrt(variance): 778.033
sqrt(variance/n): 50.0138
sqrt(err1*err1 + err2*err2): 143.944
(mean2-mean1)/sqrt(err1*err1 + err2*err2): -1.22317
```
Here the average journey time is 3 minutes and 38 seconds plus or minus 13 seconds.

Lastly we re-run *model* 27 in mode 5,
```
gazebo -u --verbose ~/turtlebot3_ws/src/TBOT01_ws/env009.model -s libgazebo_ros_init.so

```

```
ros2 run TBOT01 controller data016.bin 250 5000

```
```
ros2 run TBOT01 actor model027 room5 1000 data009 5 mode005 0.05 

```

```
ros2 run TBOT01 commander room5 250

```
```
./main room_expected data011 room5 17 data016
...
dataset: data011
z: 42400
n: 33
counts: [211,558,1434,843,1469,4057,1193,3029,1015,987,2413,823,398,384,648,290,172,3976,1953,212,5581,496,1606,103,2611,1226,66,888,895,878,1127,21,559]
mean: 1276.42
sqrt(variance): 1277.26
sqrt(variance/n): 222.342
dataset2: data016
z: 54186
n: 38
counts: [209,1062,1057,2963,4093,2466,1076,596,1725,1197,1462,968,414,159,448,449,2672,2020,1851,856,1910,2127,1526,212,2225,1013,361,214,519,1762,1950,141,1877,867,444,3331,2492,310]
mean: 1342.74
sqrt(variance): 975.122
sqrt(variance/n): 158.186
sqrt(err1*err1 + err2*err2): 272.871
(mean2-mean1)/sqrt(err1*err1 + err2*err2): 0.243018
```
The average journey time increases a little to 5 minutes and 35 seconds, but this is not significantly different from the mode 3 run of *model* 27. A much longer run would be required to demonstrate any difference.

This table summarises the results,

model|mode|rooms|mean|std err
---|---|---|---|---
none|random|55|3129|454
model028_location|1|75|2503|341
model028_location|2|60|2409|311
model028_location|3|43|1049|134
model027|3|33|1276|222
model028_location|4|53|2057|293
model028_location|5|242|873|50
model027|5|38|1342|158


There are many possible ways to improve the performance of TBOT01. We could experiment with various parameters -

* turtlebot controller parameters, e.g. the random-turn interval, or the mode 3/5 turn threshold.  

* *substrate* parameters, e.g. the bucketing of the `scan` *values*, or the `substrate005` definition of `location`

* *modelling* parameters to improve *likelihoods* or label accuracies

* environment parameters, e.g. more furniture in rooms, or less regular shapes, to make it easier to identify location

Another method is to acquire more random-turn experience. Even without a new *model*, larger *slice histograms* would allow for more accurate action decisions. 

In addition, the datasets generated by an actor's runs could become the dataset given to a re-run of the actor recursively, as mentioned above. 

Larger random-turn or actor *histories* would allow for new *models* that have larger *decompositions* with more leaf *slices*. This is the case whether the *model* is *induced*, or partially or wholly *conditioned* on `location` or some other label *variable*.

More *history* might reveal *alignments* between the timewise *frames* of past `scan` *variables* and the past `motor` *variable*. In this way, a short term memory might also improve performance in rooms that are easily confused in the current *frame*, but were distinguishable a few *frames* ago; or where the walls are out of range of the lidar in the current *frame*, but were not a short time in the past.

Another possible improvement would be to redefine the `location` label to give it more resolution. For example, its *values* could represent (i) a house coordinate, e.g. to the nearest metre, and (ii) an orientation, e.g. to the nearest 30 degrees. The motor action decision, given the goal room, would tend to move/rotate the turtlebot to the next coordinate/orientation that is closest to the correct exit. A *model* could also be *conditioned* on the new `location` *variable*. 

Alternatively we could replace the future label, `room_next`, with a set of future room *variables* each with a *value* representing the bucketed number of *events* to attainment. Then the motor decision would be the action with the fewest steps given the desired next room. 
