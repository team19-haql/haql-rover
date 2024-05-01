# Usage

*Rover usage: everything should be automated with the configuration file. You should not need to run these commands manually*

To run any of the packages, you must source the compiled workspace first.

```bash
cd ws_dev
source install/setup.bash
```

## Launch Commands

The navigation system is broken into several packages that can be run individually.

### Bodenbot Bringup

The main launch script is located in the bodenbot package and can be run
with the following command. These should be good defaults for starting the rover.

```bash
ros2 launch bodenbot bodenbot.launch.py \
    start_controller_node:=true \
    start_navigation:=true \
    start_traverse_layer:=true \
    start_docking_server:=true \
    start_webots:=false \
    use_mock_hardware:=false \
    debug_hardware:=false
```

#### Options

- `start_controller_node` - The nodes necessary for hardware movement
- `start_navigation` - Start the navigation system.
- `start_traverse_layer` - Start the mapping system.
- `start_docking_server` - Start the docking server used for dog house docking.
- `start_webots` - Start the webots simulation.
- `use_mock_hardware` - Use mock hardware instead of the real hardware.
Useful for debugging.
- `debug_hardware` - Print debug messages from the hardware (motor) interface.

### [Simulation](./simulation.md)

Start the webots simulation

```bash
ros2 launch webots_dev robot_launch.py 
```

### Watching system from an external source (foxglove)

If you need to view the system from an external computer, I recommend installing
foxglove studio. To enable connections to the robot system, you must launch the foxglove
bridge.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Then you can connect to the computer using the foxglove application.

## Bodenbot Scripts

The [Bodenbot Scripts](./software/bodenbot_scripts.md) package contains utility, automation, and testing code for the rover. 