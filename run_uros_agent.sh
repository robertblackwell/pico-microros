#!/bin/bash
source ~/.bashrc
source install/setup.bash
echo number of args $#
echo first [$0]
for i; do
    echo $i
done

if [ -z $1  ]; then
    echo no device given defaulting to /dev/ttyACM0
    TERMDEV=/dev/ttyACM0
else
    echo device given $1
    TERMDEV=$1
fi

echo ros2 run micro_ros_agent micro_ros_agent serial --dev ${TERMDEV} baudrate=115200

ros2 run micro_ros_agent micro_ros_agent serial --dev ${TERMDEV} baudrate=115200
