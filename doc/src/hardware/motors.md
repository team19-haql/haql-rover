# Motors

The raspberry pico microcontroller is connected to each motors with an
single pulse encoder pin, a PMW output signal, and a direction signal.
We use single pulse encoding due to a limited number of pins on the board.
The PWM and direction signals are connected a dual channel motor driver board. Each motor driver board is responsible for driving 2 motors.

## Drivers

The microcontroller and rover side each have driver code which is used to communicate to each other. 
### Microcontroller side driver

The source code for the controller on the pico can be found [here](https://github.com/team19-haql/bodenbot-motor-controller).

### Rover side driver

A python version of the driver to communicate with the Raspberry pi pico for the Jetson Orin Nano can be found [here](https://github.com/team19-haql/haql-rover/blob/main/bodenbot_scripts/bodenbot_scripts/motor_controller.py). This version is much more legible. The compiled implementation used in the final launch script is part of the [bodenbot package](../software/bodenbot.md)

The bodentbot rover has 6 motors that each has independent PID control.
Motor control is mainly handled by the raspberry pi microcontroller
located on the PCB.

## Encoder

The motors are connected to the controller with a single pin making it a single pulse encoder. As a result, the motor controller doesn't actually know the direction the wheels are spin, but estimates the direction based on the commands sent to the motor for what direction it should be spinning. 

## Hardware info

We use the [NFP-5840-31ZY-EN](https://microdcmotors.com/product/12v-24v-high-torque-low-speed-dc-motor-with-encoder-self-locking-model-nfp-5840-31zy-en) from microdcmotors. We are using the 12V 500 gear ratio motors. Originally the rover was designed for 100 ratio, but needed to be upgraded to 500 due to lacking torque. 

## Communication Protocol

In the runtime for the pico, we run a PID loop for each motor.
The pico can accept motor commands from an external source via I2C.
In our case, we have connected the I2C bus to the Jetson Orin Nano
where the main rover control happens.

The pico supports 2 types of command, it has the ability to
read and write to a single byte register value.

Reading from a motor register returns a value that represent the
current measured velocity of the associated motor.

Writing to a motor register will set the target value for that motor.

### Registers

The motor registers are as follows:

- `0x00`: motor #1
- `0x01`: motor #2
- `0x02`: motor #3
- `0x03`: motor #4
- `0x04`: motor #5
- `0x05`: motor #6
- `0x10`: led #0
- `0x20`: fan #0
- `0x21`: fan #1

### USB

There are 2 main usb commands with the following format:

- `write <register> <value`
- `read <register>`

Values should be formatted as floating point numbers like so `1.555`.

### I2C

I2C supports reading and writing to registers. The numbers are expected to be
formatted as a standard 4 byte floating point number. This interface is not SMBus compatible. 