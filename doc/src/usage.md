# Usage

To run any of the packages, you must source the compiled workspace first.

```bash
cd ws_dev
source install/setup.bash
```

## Watching system from an external source (foxglove)

If you need to view the system from an external computer, I recommend installing
foxglove studio. To enable connections to the robot system, you must launch the foxglove
bridge.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Then you can connect to the computer using the foxglove application.

## Launch individual packages

The navigation system is broken into several packages that can be run individually.

### Simulation

Start the webots simulation

```bash
ros2 launch webots_dev robot_launch.py 
```

### Navigation

To run navigation, both the traversability mapping, and navigation
system must be running.

```bash
ros2 launch boden_navigation navigation_launch.py
ros2 launch traverse_layer traverse_layer_launch.py
```

### Docking Server

```bash
ros2 launch boden_bringup apriltag.launch.py
```

### Autonomous System

To run the autonomous system, the docking server and navigation
system must be running first.
Then the autonomous script can be started.

```bash
ros2 run boden_navigation demo_auto
```
