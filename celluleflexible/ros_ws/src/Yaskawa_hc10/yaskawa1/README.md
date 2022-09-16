# How to launch the project in ROS

### __On robot__ :

If you want to send command to the robot with MoveIt, a plugin on RVIZ, you just need to type this command line :
```bash
roslaunch yaskawa1 yaska1_rviz.launch sim:=false robot_ip:=192.168.1.40 controller:=yrc1000
```

# Usefull topics

### **/motoman_hc10/joint{$}_position_controller/command**

topic controlling the pose of the joint {$} in radiant.

*/!\ All joints range between [-3.141592653589793;+3.141592653589793], except joint 3 for which the pose control ranges between [0;6.28318530718] !!!*


