# BodenBot Rover Packages

## Structure

### boden_bringup

Launch scripts for starting robot services.

### boden_description

Description of the robot in URDF format.

### boden_interfaces

Communication interfaces for the rover such as the docking action request.

### boden_misc

Various nodes used in the running of the rover like the motor driver and
docking action server.

### boden_navigation

The package with configuration for the navigation stack used by the rover.

### traverse_layer

Implementation of traversability mapping used for obstacle avoidance in
uneven terrain.

### webots_dev

Package for launching the webots simulation.

## Installation

I highly recomend using ubuntu. If you are on windows, use the wsl subsystem.
If you are on mac, you are on your own.

This project was tested with ros2 humble, but it should work on humble or later.

**Install [ros2](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html)**

Make sure to source the ros2 setup file. You can copy the command into
your `~/.bashrc` file if you want it to be sourced automatically.

```bash
# should run this command before any build command
source /opt/ros/humble/setup.bash
```

Make a ros workspace and clone this repo the `src` directory.

```bash
mkdir -p ws_dev/src
cd ws_dev/src
git clone git@github.com:team19-haql/haql-rover.git
```

## Compiling

Install dependencies using `rosdep`.

```bash
# make sure you are in the root of the workspace
cd ws_dev

# install dependencies
sudo apt update
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y
```

Make sure you are in the root of the workspace and build all packages
in the project.

```bash
cd ws_dev
# make sure you have sourced a ros setup file before building
colcon build --symlink-install
```

## Usage

Source the compiled workspace.

```bash
cd ws_dev
source install/setup.bash
```

To start the simulation, run the following command.

```bash
ros2 launch webots_dev robot_launch.py 
```

To start the localization node, run the following command.
You can find different launch files in `src/haql-rover/boden_navigation/launch`

```bash
ros2 launch boden_navigation ekf_launch.py 
```
