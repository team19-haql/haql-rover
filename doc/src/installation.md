# Installation

*The physical rover should have all software preinstalled and you can look at [jetson](./hardware/jetson.md#usage) for rover specific instructions*

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

Make sure you are in the root of the workspace and have ros sourced.
Then build all packages in the project.

```bash
cd ws_dev
# make sure you have sourced a ros setup file before building
colcon build --symlink-install
```

## Usage

To run any of the packages, you must source the compiled workspace first.

```bash
cd ws_dev
source install/setup.bash
```

This simulation start the simulation, run the following command.

```bash
ros2 launch webots_dev robot_launch.py 
```

For more launch script, look in [usage](./usage.md).
