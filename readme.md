# Making a micro-ROS app for raspberrypi Pico

This repo is a demonstration of connecting a RaspberryPi Pico and an Ubuntu host using micro-Ros.

To make this connection one needs a firmware app running on the Pico and an instance of
the micro-ROS-Agent running on the Ubuntu host.

THe Pico firmware in this example contains both a publisher and subscriber.

This repo also provides a python ROS node that subscribes to the topic being published by the
Raspberrypi Pico and a python ROS node that publishes to the subscriber on the pico.

## Two things that got in the way of this demo

While makeing this demo I ran into a problem where the publisher on the pico worked
as expected right from the start but the subscriber did not receive any messages.
This situation persisted for a number of days and all attempts to debug the problem
failed. 

Eventually I found two things, that when corrected fixed the problem.

### ROS_DOMAIN_ID

The pico version of the micro-ROS code by default sets the Picos ROS_DOMAIN_ID to zero.
Be carefull to check that there is no mismatch between the pico and your host in regard
to the value of ROS_DOMAIN_ID. The pico demo code prints out the picos domain_id value.

### std_msgs__msg_String

While setting up the code and variables for the pico's subscriber I decided to
use `std_msgs__msg__String` as the ros type of the message that the subscriber would receive.

I declared a variable of this type at global scope so that the subscriber callback could
access it; which actually is unnecessary but in any case thats what I did.

```c
std_msgs__msg__String sub_msg;
```

This declaration fails to initialize the variable correctly filling it with zeros. 

For some messages types, say `std_msgs__msg__Int32`, the zero fill works out to be acceptable.

But for `std_msgs__msg__String` it is inadequate as this type contains a pointer.
Its details are something like:

```c++
struct std_msgs__msg__String {
    struct {
        size_t size;
        size_t capacity;
        char *  data;
    }
}

```
The result of my declaration and default initialization  leaves the inner `data` pointer with a value of NULL.

Thus there is nowhere for the lower level `microros` to store incoming messages. Hence message reception fails
silently.

The solution:

```c

std_msgs__msg__String sub_msg;
char sub_buffer[512];
....
....
int main() {
    ....
    ....
    sub_buffer[0] = '\0';
    sub_msg.data.data = &sub_buffer[0];
    sub_msg.data.capacity = 511;
    sub_msg.data.size = 0;

```
## A note on uarts and usb ports

As currently configured the Pico firmware will connect with the micro-RoS Agent on the usb port using stdio functions but with CR/LF processing disabled.

The connection between the firmware and the micro-ROS agent __can__ be configured to operate on `uart0` using stdio function (on pins `GPIO16` `GPIO17`) by a change to
the CMakeLists.txt file. See comments in that file.

If you look at the code in uros_main.cpp you will notice there are tracing printouts in the form of `FTRACE(....)` in the code. These are
implemented in `src/common/trace.cpp` and `src/common/trace.h` and are sent to `uart1` which is exposed on pins `GPIO8` and `GPIO9` and cannot/do not use
standard stdio functions.

Why the non-standard pins ? A constraint of the hardware layout of the actual project I am working on.

## The pico firmware 

The pico firmware used in this demonstration is derived from the repo  
[https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk).


Its worth noting that this demonstration does not use any custom Ros messages. The use of custom messages
introduces additional complexities that will be tackled in a future demonstration.

## Getting started

This example has a number of components some of which are in the repo and some of which will be downloaded
or cloned from other repos as we work through the build process.

Get this repo onto your machine:

```bash
git clone -b master git@github.com/robertblackwell/pico-microros.git
```

## What do we have

The sub directory `pico-pubsub` contains a full pico firmware project that includes
the pico based micro ros components. This is build with `cmake` in the usual manner.

```bash
mkdir build && cd build
cmake ..
make -j 8
```
and from the build directory the command

```bash
cp pico-single-publisher /media/<username>/RPI-RP2
```
will load the firmware onto a connected pico.

Note this is all assume Ubuntu 22.04 as the host.


## micro-ROS-Agent

We now need to install and run the micro-ROS-Agent. The agent is not part of this repo
but will be downloaded and built in what follows.

If you look on the internet you will find that
we can acquire the agent with a `docker` image or on Ubuntu by using `snapd`. Below is a brief outline of those
mechanisms and a reference for each.

However for this demonstration we are going to setup and run the agent without either of those tools
in the interest of understanding.

### micro-ROS-agent 'by hand'

This approach will use the contents of the repo [https://github.com/micro-ROS/micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup).

Before following the instructions in the readme.md of the micro_ros_setup repo make sure you have the following:

-   a top level directory called something like `pico-microros`
-   a sub directory called `pico-pubsub`  which is derived from a clone of `https://github.com/micro_ros_raspberrypi_pico_sdk`
-   a sub directory called `src` which contains our python ROS2 package `py_subscriber`

This structure is a ROS workspace. 

#### micro-ros-setup package

Make sure the env variable $ROS_ID is set, for me it is "iron" and then add the ROS package `micro_ros_setup` to this workspace with the commands 


```bash
source /opt/ros/$ROS_ID/setup.bash
git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

```

#### micro-ROS-Agent package

The commands in the previous section have build the micro-ros-setup package which provides tools that can build 
a micro-ros agent. Soo now execute

 ```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

 ```
These commands will have added a sub directory `./src/uros` which contains the microros agent repo.

All the pieces are in place to mow run the micro ros agent with the command

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
__/pico_publisher_topic__
__/pico_subscriber_topic__
/rosout

No check the pico code is publishing messages:

```bash
ros2 topic echo /pico_publisher_topic
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

Finally lets build our ROS2 python package py_subscriber and monitor the output from the Pico.

From the project roote directory install the dependencies for the python package with"

```bash
rosdep install -i --from-paths src --rosdistro iron -y
```
Now build the python package:

```bash
colcon build --packages-select py_subscriber
```

Now run that package with:

```bash
ros2 run py_subscriber listener

```

You should see output.

see [https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico) for more details.

## References

The following references are worth reading.

[https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3](https://robofoundry.medium.com/raspberry-pi-pico-ros2-via-micro-ros-actually-working-in-1-hr-9f7a3782d3e3)

[https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico)