# Depth Camera

The depth camera we use is the [Zed2i](https://www.stereolabs.com/products/zed-2) from stereolabs. This is a stereo camera that can estimate the depth of an image (how far a pixel is) based on the same principle as humans with 2 eyes. 

One of the advantages of the zed2i are its accuracy compared to alternatives. The accuracy of depth estimation is important for generation our [navigation map](../software/traverse_layer.md).

Stereolabs provides an [SDK](https://www.stereolabs.com/docs/get-started-with-zed) for the zed2i which handles most of the interactions and complex algorithms with the camera like depth estimation and [VIO](#vio)

## Sensors

The camera has a couple of sensors that do various things, but these are the 2 we mostly care about.

- Imu - used for localization
- Stereo camera - used for spatial tracking and mapping

## VIO

VIO is short for Visual Inertial Odometry. This is a more sophiticated version of visual odometry. Visual odometry is the idea that we can estimate the movement between 2 photos with an algorithm. By then adding inertia (velocity) from another sensor, we can increase the accuracy of that movement estimation. 

In short, VIO is a way to estimate movement using a camera. 