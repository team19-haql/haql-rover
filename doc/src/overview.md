# Overview

Welcome to the documentation of the bodenbot rover. This document aims to be a guide in the usage of the bodenbot software, and also as an aid to any future developers. If you are interested in the more technical parts of this documentation, an understand of ros and basic robotics concepts is expected. 

## Installation and usage

Before running anything, make sure that your workspace is setup. Visit [installation](./installation.md) for details of how to setup the software. How to use the software is found in [usage](./usage.md). 

If you are using the Jetson Orin Nano, the software should already be setup for usage, in which please checkout the [jetson](./hardware/jetson.md#usage) page. 

Information about configuring the automation system should be in [configuration](./configuration.md)

## [Hardware](./hardware/hardware.md)

The hardware chapter has brief information of the hardware involved in the rover from a software perspective. 

## Software

The software chapters describe the software packages used for bodenbot. 

### [Simulation](./simulation.md)

The we use [webots](https://cyberbotics.com/) for simulation. The model of the rover used for simulation is generated from an earlier revision of the CAD model so there are some differences. Mainly, the orientation of the suspension is the reverse of what is found on the final rover. The limits set on the rover are also different, so the max speed of the rover in simulation is much faster. 

### Navigation

There are 3 main components to the navigation system. Localization, mapping, and navigation.

For localization we fuse a few sources of information:
- [Wheel Encoders](./hardware/motors.md#encoder)
- [Zed2i VIO](./hardware/depth_camera.md#vio)
- [GPS](./hardware/sensors.md#gps)

The [mapping](./software/traverse_layer.md) uses a traversability algorithm to generate a 2d map. Each cell of this 2d map represents whether the rover would be able to drive over that area. 

The navigation stack is handled by the [navigation2](https://navigation.ros.org/) framework.

Details of the data flow are [here](./navigation.md).

## Naming

Bodenbot is a placeholder name that was only ever used in software, the physical rover to this day has no proper name. Due to the fact that this is mostly software documentation, the placeholder name is quite prevalent. 