# Sensors

### GPS

The rover has a GT-U7 gps module installed in the electronics box. It is connected to the [jetson nano](./jetson.md) using the UART1 pins on the 40 pin header. These UART pins connect to the `/dev/ttyTHS0` serial port. The data is in NMEA format.

To connect the GPS to the ros2 system, we use the [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2).
### [Depth Camera](./depth_camera.md#sensors)

We use the zed2i camera from stereo labs for the depth camera.

### [Encoders](./motors.md#encoder)

The encoders are integrated into the motors. 