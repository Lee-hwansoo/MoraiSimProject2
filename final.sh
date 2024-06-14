#!/bin/bash

roslaunch auto_driving final.launch & FISRT_LAUNCH_PID=$!

wait $FISRT_LAUNCH_PID

roslaunch auto_driving parking_enter.launch
