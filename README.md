# ROS-TurtleBot-PID
This project demonstrates the simulation of ROS Turtlebot3 path tracking with PID. I have generalized the pid controller to track circular or linear trajectories.
Here is the youtube [video](https://youtu.be/okqIgZJy67E).

## Instructions to run the project
* First, open a terminal(Lets call terminal 1) and navigate the project.
* Next, go to the root directory(terminal 1):
 ```cd ~/catkin_ws/src/ # (you need to create the dir first if none exists, see ROS tutorial http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)```
* Then, clone following git projects(terminal 1):
```git clone ​ https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git```
```git clone ​ https://github.com/ROBOTIS-GIT/turtlebot3.git```
```git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git```
* Then, use the following command(terminal 1):
```source devel/setup.bash```
* Then, use the following command(terminal 1):
```cd ~/catkin_ws && catkin_make```
* Then, Set the turblebot3 model(terminal 1),
```gedit ~/.bashrc```
* Then, Set the turblebot3 model(terminal 1):
```export TURTLEBOT3_MODEL=waffle```
* Then, Save this file, and close it. Then make it to take effect immediately(terminal 1),
```source ~/.bashrc```
* Next, go to the src directory of your workspace (/catkin_ws/src).
* Next, download the “tb3_simulation” package from Canvas and unzip it. then put this “tb3_control” folder under and copy and paste it to /catkin_ws/src.
* Then, use the following command(terminal 1):
```cd ~/catkin_ws && catkin_make```
* Next, open a separate terminal, source it and run following command(terminal 2) and keep it opened:
```roslaunch turtlebot3_fake turtlebot3_fake.launch```
* Next, run the rrt project using following command(terminal 1):
```roslaunch tb3_control control.launch```
* Find the “Marker” and add a "Marker". Now Rviz should start to listen to the Marker messages
published from the control node, and you should see a screen something similar to the attached [video](https://youtu.be/okqIgZJy67E).

