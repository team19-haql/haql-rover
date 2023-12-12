# Installation
I highly recomend using ubuntu. If you are on windows, use the wsl subsystem.
If you are on mac, you are on your own.

This project was tested with ros2 iron, but it should work on iron or rolling.

Install [ros2](https://docs.ros.org/en/iron/Installation/Alternatives/Ubuntu-Install-Binary.html)
Make sure to source the ros2 setup file. You can copy the command into your `~/.bashrc` file if you want it to be sourced automatically.
```bash
source /opt/ros/iron/setup.bash
```


Make a ros workspace and clone this repo the `src` directory.
```bash
mkdir -p ws_dev/src
cd ws_dev/src
git clone git@github.com:team19-haql/haql-rover.git
```


# Compiling
Install dependencies using `rosdep`.
```bash
# make sure you are in the root of the workspace
cd ws_dev
# install dependencies
sudo apt update && sudo rosdep update && sudo rosdep install --from-paths src --ignore-src -y
```


Make sure you are in the root of the workspace and build all packages in the the project.
```bash
cd ws_dev
colcon build
```
# Usage
Source the compiled workspace.
```bash
cd ws_dev
source install/setup.bash
```


To start the simulation, run the following command.
```bash
ros2 launch webots_dev robot_launch.py 
```

To start the localization node, run the following command. You can find different launch files in `src/haql-rover/boden_bringup/launch`
```bash
ros2 launch boden_bringup ekf_launch.py 
```