# LeddarTech Vu8 ROS Driver [![Build Status](https://travis-ci.org/dispatch-ai/ros-leddar-vu8.svg?branch=master)](https://travis-ci.org/dispatch-ai/ros-leddar-vu8)

ROS package that configures and communicates with the [LeddarTech Vu8](http://leddartech.com/modules/leddarvu/) over [CAN](https://www.kernel.org/doc/Documentation/networking/can.txt).

## build

Clone it to e.g. `~/code/ros-leddar-vu8`:

```bash
$ git -C ~/code clone git@github.com:dispatch-ai/ros-leddar-vu8.git clone git@github.com:dispatch-ai/ros-leddar-vu8.git
```

Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) if you don't have one already:

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

add this package to it:

```bash
$ ln -s ~/catkin_ws/src/src/leddar_vu8 ~/code/ros-leddar-vu8
```

and finally build:

```bash
$ catkin_make
```

## run

Connect your [Leddar Vu8](http://leddartech.com/modules/leddarvu/) over CAN (e.g.
using a [USB/CAN adapter](http://www.peak-system.com/PCAN-USB.199.0.html?L=1)) and setup a network interface for it, e.g.:

```bash
$ sudo ip link set can0 up type can bitrate 1000000
$ sudo ifconfig can0 up
```

Then launch the driver along w/ rviz to see points:

```bash
$ roslaunch leddar_vu8 leddar_vu8.launch --screen
```

## params

These are static [private parameters](http://wiki.ros.org/Parameter%20Server#Private_Parameters)
you can define when starting the driver:

* `interface` CAN network interface, defaults to `can0`.
* `send_timeout` socket send timeout, defaults to `0` (i.e. no timeout).
* `receive_timeout` socket receive timeout, defaults to `0` (i.e. no timeout).
* `stream_timeout` socket receive timeout when streaming detections, defaults to `0` (i.e. no timeout).
* `min_range` minimum detection distance in meters, defaults to `0` (i.e. no minimum).
* `max_range` maximum detection distance in meters, defaults to `20`.
* `fov` detection field-of-view in degrees, defaults to `100`.
* `rate` frequency at which to publish detection scans in hertz, defaults to `50`.
* `frame_id` name of the [tf](http://wiki.ros.org/tf) frame to use when publishing detection scans, defaults to `laser`.
* `base_rx_message_id` base id of CAN messages sent to sensor, defaults to `0x740`.
* `base_tx_message_id` base id of CAN messages received from sensor, defaults to `0x750`.

There are also [dynamic parameters](http://wiki.ros.org/dynamic_reconfigure) defined
in [Config.cfg](cfg/Config.cfg) that can be set using e.g.:

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

## develop

You can create e.g. an [Eclipde CDT](http://www.eclipse.org/cdt/) project for it:

```bash
$ mkdir ~/code/ros-leddar-vu8-project
$ cd ~/code/ros-leddar-vu8-project
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CATKIN_ENABLE_TESTING=ON ~/code/ros-leddar-vu8
```

and then install [CAN utils](https://github.com/linux-can/can-utils):

```bash
$ sudo apt-get install can-utils
```

so you can e.g. monitor interface traffic:

```bash
$ candump can0,0:0,#FFFFFFFF -ace
```
