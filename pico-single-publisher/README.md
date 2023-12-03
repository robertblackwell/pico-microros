# Raspberrypi Pico Micro-ROS Firmware

This project contains 

-   firmware for a Pico that contains a single ROS2 publisher.
-   instuctions for building the MicroRos Agent

It is a copy of `https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)` with a few mods
and more explanation.

Following the instructions below will also build and run the micro-ROS Agent that is required for
a pico to communicate with an Ubuntu host.



# Making a micro-ROS app for raspberrypi Pico

This repo is a demonstration of connecting a RaspberryPi Pico and an Ubuntu host using micro-Ros.

To make this connection one needs a firmware app running on the Pico and an instance of
the micro-ROS-Agent running on the Ubuntu host.

For this experiment we are going to use a demonstration Pico app residing in the repo 
[https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk).

Its worth noting that this demonstration does not use any custom Ros messages. The use of custom messages
introduces additional complexities that will be tackled in a future demonstration.

## before we start

This example will have a number of components and it is worth making a container directory for all these
components before we start.

So create a new empty directory.

```bash
mkdir -p pico-micro-ros-example/src

```

## micro_ros_paspberrypi_pico_sdk firmware

This app can be downloaded and built using the following commands:

```bash
cd pico-micro-ros-example
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk
cd micro_ros_raspberrypi_pico
mkdir build
cd build
cmake ..
make -j 10

```

and (on an Ubuntu host) the image can be loaded onto a Pi Pico with the command:

```bash
cp pico-micro_ros_example /media/<user>/RPI-RP2
```

The above assumes that you already have the RaspberryPi Pico SDK setup and can build pico firmware apps.

When complete your directory structure should be
<pre>
    pico-micro-ros-example--+
                            |
                            +---micro_ros_raspberrypi_pico_sdk
                            |
                            +---src
</pre>

and the `src` directory is at this pint empty.

## micro-ROS-Agent

We not need to install and run the micro-ROS-Agent. If you look on the internet you will find that
this can be done with a `docker` image or on Ubuntu using `snapd`. Below is a brief outline of those
mechanisms and a reference for each.

However for this demonstration we are going to setup and run the agent without either of those tools
in the interest of understanding.

### micro-ROS-agent 'by hand'

This approach will use the contents of the repo [https://github.com/micro-ROS/micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup).

Before following the instructions in the readme.md of the micro_ros_setup repo make sure you have the following:

-   a top level directory called something like `pico-micro-ros-example`
-   a sub directory called `micro_ros_raspberrypi_pico_sdk`  which is a clone of `https://github.com/`
-   a sub directory called `src` which is empty.

This structure is a ROS workspace. 

#### micro-ros-setup package

Now add the ROS package `micro_ros_setup` to this workspace with the commands

Make sure $ROS_ID is set, for me it is "iron"

```bash
source /opt/ros/$ROS_ID/setup.bash
git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

```

#### micro-ROS-Agent package

The commands in the previous section have build the micro-ros-setup package which provides tools that can build 
a micro-ros agent. 

 ```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

 ```
These commands will have added a directroty `./src/uros` which contains the miro ros agent repo.

All the pieces are in place to mow run the micro ros agen with the command

 ```bash
 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200
 ```

#### Check its working

With the agent running on a host, the micro-ros-example firmware loaded onto the pico, we should now be able to check
that its all working.

First check - the code on the pico has registered as a publisher:

```bash
ros2 topic list
```

output should be:


/parameter_events
__/pico_publisher__
/rosout

No check the pico code is publishing messages:

```bash
ros2 topic echo /pico_publisher
```

should produce something like:

```
data: 22
---
data: 23
---
data: 24
---
data: 25
---
data: 26
---
data: 27

```

### Docker

```bash
docker pull microros/base:iron
docker run -it --rm --net=host microros/micro-ros-agent:iron
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:iron serial --dev /dev/ttyACM0 -b 115200
```

see [https://hub.docker.com/r/microros/micro-ros-agent](https://hub.docker.com/r/microros/micro-ros-agent) for more details.

### snapd

Make sure the pico plugged into a USB port then execute these commands

```bash
sudo snap install micro-ros-agent
sudo systemctl restart snapd
snap interface serial-port
snap connect micro-ros-agent:serial-port snapd:pico

```

Now actually run the micro ros agent

```bash
micro-ros-agent serial --dev /dev/ACM0 baud=115200
```

## A simple package to monitor the Pico Publisher



see [https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico) for more details.

## References

The following references are worth reading.

[https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3](https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3)

[https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico)