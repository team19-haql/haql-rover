# Installation

*The physical rover should have all software preinstalled and you can look at [jetson](./hardware/jetson.md#usage) for rover specific instructions*

I highly recomend using ubuntu. If you are on windows, use the wsl subsystem.
If you are on mac, you are on your own.

This project was tested with ros2 humble, but it should work on humble or later.

## Zigmaps

One part of the mapping system was made in a separate [zigmaps](./software/traverse_layer.md#zigmaps) library. This needs to be installed before the rest of the system. In order to build the library, [zig](https://ziglang.org/) needs to be installed. 

```bash
git clone https://github.com/deweykai/zigmaps
cd zigmaps
# install to the system
sudo zig build --release=fast -p /opt/local
```

## Install [ros2](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html)

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

> **Navigation MPPI** not available for ros2 humble.
> 
> We rely on the MPPI controller which is relatively new. At the time of writing this it could not be downloaded for ros2 humble using `apt`. We can get just this directory from the nav2 repository and clone it ourselves to solve the issue.
> ```bash
> git clone https://github.com/open-navigation/navigation2
> mv navigation2/nav2_mppi_controller .
> rm -rf navigation2
> ```

Install dependencies using `rosdep`.

```bash
# make sure you are in the root of the workspace
cd ws_dev

# install dependencies
sudo apt update
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y
sudo apt install libi2c-dev
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
