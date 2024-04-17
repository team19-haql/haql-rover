# Motors

The source code for the controller on the pico can be found [here](git@github.com:team19-haql/bodenbot-motor-controller.git).

A driver to communicate with the Raspberry pi pico for the Jetson Orin Nano can be found [here](https://github.com/team19-haql/haql-rover/blob/main/boden_misc/boden_misc/motor_controller.py).

The bodentbot rover has 6 motors that each has independent PID control.
Motor control is mainly handled by the raspberry pi microcontroller
located on the PCB.

The raspberry pico microcontroller is connected to each motors with an
single pulse encoder pin, a PMW output signal, and a direction signal.
We use single pulse encoding due to a limited number of pins on the board.
The PWM and direction signals are connected a dual channel motor driver board. Each motor driver board is responsible for driving 2 motors.

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
formated as a 4 byte fixed point number (`I16F16` in the `fixed` crate).
Numbers are in little endian format.
