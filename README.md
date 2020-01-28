# TBOT01
TurtleBot3 controller

http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

https://index.ros.org/doc/ros2/

```
cd /d C:\zzz\caiks
mkdir AlignmentC_build AlignmentRepaC_build TBOT01_build
cd /d TBOT01_build
"C:\Program Files\CMake\bin\cmake" -G "Visual Studio 14 2015" -A x64 ../TBOT01
"C:\Program Files\CMake\bin\cmake" --build . --config Release

"C:\Program Files\CMake\bin\cmake" --build . --config Release --target TBOT01

cd /d C:\zzz\caiks
mkdir TBOT01_ws
cd /d TBOT01_ws

..\TBOT01_build\Release\TBOT01.exe

..\TBOT01_build\Release\TBOT01.exe stats 202001222010_2.TBOT01

..\TBOT01_build\Release\TBOT01.exe stats 202001271320_room1.TBOT01

..\TBOT01_build\Release\TBOT01.exe bitmap 202001222010_2.TBOT01

..\TBOT01_build\Release\TBOT01.exe bitmap 202001222010_2.TBOT01 3

..\TBOT01_build\Release\TBOT01.exe bitmap 202001271320_room1.TBOT01

..\TBOT01_build\Release\TBOT01.exe induce model001 4

```
