# TBOT01
TurtleBot3 controller

http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

https://index.ros.org/doc/ros2/

```
cd /d C:\zzz\caiks
# move TBOT01\CMakeLists_noros.txt TBOT01\CMakeLists.txt
mkdir AlignmentC_build AlignmentRepaC_build TBOT01_build
cd /d TBOT01_build
"C:\Program Files\CMake\bin\cmake" -G "Visual Studio 14 2015" -A x64 ../TBOT01
"C:\Program Files\CMake\bin\cmake" --build . --config Release

"C:\Program Files\CMake\bin\cmake" --build . --config Release --target main

cd /d C:\zzz\caiks
mkdir TBOT01_ws
cd /d TBOT01_ws

..\TBOT01_build\Release\main.exe

..\TBOT01_build\Release\main.exe stats 202001222010_2.TBOT01

..\TBOT01_build\Release\main.exe stats 202001271320_room1.TBOT01

..\TBOT01_build\Release\main.exe bitmap 202001222010_2.TBOT01

..\TBOT01_build\Release\main.exe bitmap 202001222010_2.TBOT01 3

..\TBOT01_build\Release\main.exe bitmap 202001271320_room1.TBOT01

..\TBOT01_build\Release\main.exe bitmap_average 202001222010_2.TBOT01

..\TBOT01_build\Release\main.exe bitmap_average 202001271320_room1.TBOT01 20

..\TBOT01_build\Release\main.exe bitmap_average 202001271320_room4.TBOT01 20

..\TBOT01_build\Release\main.exe induce model001 4 >model001.log

..\TBOT01_build\Release\main.exe bitmap_model model001 

..\TBOT01_build\Release\main.exe induce model002 4 >model002.log

..\TBOT01_build\Release\main.exe bitmap_model model002 

..\TBOT01_build\Release\main.exe induce model003 4 >model003.log

..\TBOT01_build\Release\main.exe bitmap_model model003 

..\TBOT01_build\Release\main.exe induce model004 4 >model004.log

..\TBOT01_build\Release\main.exe bitmap_model model004 

..\TBOT01_build\Release\main.exe induce model005 4 >model005.log

..\TBOT01_build\Release\main.exe bitmap_model model005 

..\TBOT01_build\Release\main.exe condition model006 4 motor >model006_motor.log

..\TBOT01_build\Release\main.exe bitmap_model model006_motor 

..\TBOT01_build\Release\main.exe test model006_motor motor 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 890
frund(*dr->fud)->size(): 97
frvars(*dr->fud)->size(): 1113
model: model006_motor
label: motor
effective size: 6054
matches: 5849

..\TBOT01_build\Release\main.exe condition model006 4 location >model006_location.log

..\TBOT01_build\Release\main.exe bitmap_model model006_location

..\TBOT01_build\Release\main.exe test model006_location location 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 890
frund(*dr->fud)->size(): 99
frvars(*dr->fud)->size(): 1115
model: model006_location
label: location
effective size: 6054
matches: 5332

..\TBOT01_build\Release\main.exe condition model006 4 position >model006_position.log

..\TBOT01_build\Release\main.exe bitmap_model model006_position

..\TBOT01_build\Release\main.exe test model006_position position 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 890
frund(*dr->fud)->size(): 107
frvars(*dr->fud)->size(): 1123
model: model006_position
label: position
effective size: 6054
matches: 5572

..\TBOT01_build\Release\main.exe condition model007 4 motor >model007_motor.log

..\TBOT01_build\Release\main.exe bitmap_model model007_motor 

..\TBOT01_build\Release\main.exe test model007_motor motor 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 128
frund(*dr->fud)->size(): 331
frvars(*dr->fud)->size(): 2792
model: model007_motor
label: motor
effective size: 6054
matches: 5669

..\TBOT01_build\Release\main.exe condition model007 4 location >model007_location.log

..\TBOT01_build\Release\main.exe bitmap_model model007_location

..\TBOT01_build\Release\main.exe test model007_location location 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 128
frund(*dr->fud)->size(): 306
frvars(*dr->fud)->size(): 2045
model: model007_location
label: location
effective size: 6054
matches: 4864

..\TBOT01_build\Release\main.exe condition model007 4 position >model007_position.log

..\TBOT01_build\Release\main.exe bitmap_model model007_position

..\TBOT01_build\Release\main.exe test model007_position position 
hr->dimension: 363
hr->size: 6054
frder(*dr->fud)->size(): 128
frund(*dr->fud)->size(): 318
frvars(*dr->fud)->size(): 2157
model: model007_position
label: position
effective size: 6054
matches: 5213

..\TBOT01_build\Release\main.exe induce model008 4 >model008.log

```
