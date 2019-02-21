#!/bin/bash

xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/World/walls.world ; roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; TURTLEBOT_3D_SENSOR=asus_xtion_pro ; TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/catkin_ws/src/World/map.yaml ; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch pick_objects pick_objects.launch " &
