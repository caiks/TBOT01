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

..\TBOT01_build\Release\TBOT01.exe bitmap_average 202001222010_2.TBOT01

..\TBOT01_build\Release\TBOT01.exe bitmap_average 202001271320_room1.TBOT01 20

..\TBOT01_build\Release\TBOT01.exe bitmap_average 202001271320_room4.TBOT01 20

..\TBOT01_build\Release\TBOT01.exe induce model001 4 >model001.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model001 

..\TBOT01_build\Release\TBOT01.exe induce model002 4 >model002.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model002 

..\TBOT01_build\Release\TBOT01.exe induce model003 4 >model003.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model003 

..\TBOT01_build\Release\TBOT01.exe induce model004 4 >model004.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model004 

..\TBOT01_build\Release\TBOT01.exe induce model005 4 >model005.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model005 

..\TBOT01_build\Release\TBOT01.exe condition model006 4 >model006.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model006 

..\TBOT01_build\Release\TBOT01.exe condition model007 4 >model007.log

..\TBOT01_build\Release\TBOT01.exe bitmap_model model007 

```
