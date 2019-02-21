#!/bin/bash

xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/World/walls.world ; roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; TURTLEBOT_3D_SENSOR=kinect ; roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=/home/workspace/catkin_ws/src/World/gmapping.launch.xml" &
sleep 5
xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
#xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch turtlebot_teleop keyboard_teleop.launch " &
xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch wall_follower wall_follower.launch " &


#xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch slam_project apt_world.launch " &
#sleep 5
#xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch slam_project rviz.launch " &
#sleep 5
#xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch wall_follower keyboard_teleop.launch " &
#sleep 5
#xterm  -e "cd /home/workspace/catkin_ws; source devel/setup.bash ; roslaunch slam_project gmapping.launch " &
