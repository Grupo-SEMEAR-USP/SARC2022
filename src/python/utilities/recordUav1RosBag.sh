#!/bin/bash


echo "Recording topics"

rosbag record -e "/uav1/control_manager/(.*)|/uav1/odometry/odom_(.*)|/uav1/odometry/uav_state|/uav1/rgbd_down/color/image_raw|/uav1/rgbd_down/color/image_raw/theora|/uav1/rgbd_down/color/image_raw/theora/(.*)"