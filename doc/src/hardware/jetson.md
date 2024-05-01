# Jetson Orin Nano

The Jetson Orin Nano is our main controller. It orchestrates the functionality of the rest of the electronics. 

The rational behind choosing the Jeston Orin Nano was compatibility with the [zed2i camera](./depth_camera.md). The camera requires an nvidia GPU and enough processing power. We tested the original Jetson Nano, but found it lacked the necessary processing power. 

We added a small display which can be used to access the device while it is completely offline. To transfer files to and from the jetson, it is best if it and the computer that you are transfering files from are on the same network. While the rover is left outside, it might be best to disconnect the display to avoid exposure to the elements. 

## Usage

Before doing anything, make sure that the jetson has power. This should be done on the rover by making sure the top breaker is closed and the emergency power is not running.

To control the Jetson Orin Nano, it is best to be familiar with the terminal and ssh. 

*TODO: Describe connection*

*TODO: Describe Configuration*

Ideally, interacting with the rover should be as simple of modifying a configuration file. Actual operation of the rover will just read the config and act appropriately. 

*TODO: Describe isaac sim details [link](../software/ros.md)*

## Troubleshooting

*TODO: Add trouble shooting stuff*

## Hardware Details
The Jetson is configured to have a max draw of 15W. It may be slightly more in order to power peripherals like the [camera](./depth_camera.md).

### 40 pin header

| PIN      | Function     |
| -------- | ------------ |
| UART1 RX | GPS RX       |
| UART1 TX | GPS TX       |
| GPIO1    | Sensor reset |
| I2C1_SDA | I2C Bus SDA  |
| I2C1_SCL | I2C Bus SCL  |
