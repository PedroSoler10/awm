#!/bin/bash
gnome-terminal --tab -- roscore
sleep 2
gnome-terminal --tab -- roslaunch awm simulation_slam_navigation.launch
sleep 2
gnome-terminal --tab -- rosrun awm brain
