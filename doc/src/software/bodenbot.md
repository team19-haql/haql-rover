# Bodenbot

[source](https://github.com/team19-haql/haql-rover/tree/main/bodenbot)

This is the main package for running the rover. It holds the configuration for the navigation system, launch scripts for starting all the systems, and the hardware interface for the motors. 

## Ros2Control

To interface with the motors we the [ros2 control](https://control.ros.org/) framework. This communications with the motor controller with I2C as described in [motors](../hardware/motors.md#drivers).