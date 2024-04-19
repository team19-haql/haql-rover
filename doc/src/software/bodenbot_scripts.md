# Bodenbot Scripts

[source](https://github.com/team19-haql/haql-rover/tree/main/bodenbot_scripts)

The bodenbot script package is home to several scripts used in the operation
of the rover. These should not be called directly, but are used
by `Bodenbot Bringup` or for testing.

- `loader` - Honestly no idea what this does.
- `mock_battery` - A test script for battery functionality.
- `motor_driver` - A test script for motor interface
- `docking_server` - Script that is used to automate docking.
- `calibrate_striaght` - A calibration script for linear movement.
- `calibrate_box` - A calibration script for turning motor turning.
- `gps_wpf` - A test script for gps movement.
- `waypoint_recorder` - A utility script for recording waypoints in map space.
Used for testing.
- `demo_auto` - A test script for automation that became the actual
automation script.