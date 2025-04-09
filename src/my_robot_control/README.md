# Control ROS2 Package
A ROS2 package for controlling the robot using various input methods such as joystick and keyboard, with twist multiplexing for prioritized control. It also includes an emergency stop feature for safety.


## Overview
This package provides:
- Robot control via keyboard using `teleop_twist_keyboard`
- Robot control via joystick using `teleop_twist_joy`
- Emergency stop functionality via the `safety_stop_node`
- Integration with control command from the custom ros2-pkg `hand_gesture_control`
- Integration with control command from the custom ros2-pkg `web_based_control`
- Integration with navigation command from `nav2`
- Twist multiplexing to prioritize control sources from various inputs



## Usage
1. **Launch with default settings:**
```bash
ros2 launch my_robot_control my_robot_control.launch.py
```

2. **Launch with joystick and keyboard control:**
```bash
ros2 launch my_robot_control my_robot_control.launch.py use_teleop_joy:=true
```

3. **Launch with specified paramters:**
```bash
# Launch with custom frame_id for TwistStamped messages
ros2 launch my_robot_control my_robot_control.launch.py twist_stamper_frame_id:=...
```
4. **Available Launch Parameters:**

| Parameter                | Description                                      | Default Value     |
|--------------------------|--------------------------------------------------|-------------------|
| `use_teleop_keyboard`      | Enables/disables keyboard control         | `true`            |
| `use_teleop_joy` | Enables/disables joystick control              | `false` |
| `use_twist_stamper`      | Enables/disables the twist_stamper node         | `true`            |
| `twist_stamper_frame_id`           | 	Frame ID for TwistStamped message   | `base_link`            |


## Emergency Stop
The `safety_stop_node` provides an emergency stop feature to ensure safety during robot operation. It listens for specific keyboard inputs or joystick button presses to activate or deactivate the emergency stop.

### Activation and Reset
- **Keyboard**:
  - Press `insert` to activate the emergency stop.
  - Press `delete` to reset the emergency stop.
- **Joystick**:
  - Button 6 (BACK) activates the emergency stop.
  - Button 7 (START) resets the emergency stop.

### Behavior
When the emergency stop is activated:
- All robot motion is halted immediately.
- The `twist_mux` node prioritizes the emergency stop signal with the highest priority (`255`).


## Twist Multiplexing
The package uses `twist_mux` to prioritize control commands from different sources. The priority order is:
1. emergency stop - `priority` : `255`
1. Web teleoperation - `priority` : `150`
2. Keyboard control - `priority` : `100`
3. Joystick control - `priority` : `80`
4. the custom ros2-package hand_gesture_control - `priority` : `60`
5. nav2 - `priority` : `40`

Priority levels and timeouts can be configured in `config/twist_mux.yaml`.

![ROS2 Topic Graph](images/rosgraph.png)

## Package Structure

```yaml
└── 📁my_robot_control
    └── 📁config
        └── joystick.yaml
        └── teleop_twist_keyboard.yaml
        └── twist_mux.yaml
    └── 📁images
        └── rosgraph.png
    └── 📁launch
        └── my_robot_control.launch.py
        └── teleop_twist_keyboard.launch.py
    └── 📁my_robot_control
        └── __init__.py
        └── safety_stop_node.py
    └── 📁resource
        └── my_robot_control
    └── 📁test
    └── package.xml
    └── README.md
    └── setup.cfg
    └── setup.py
```

## Components

### Topics

`/cmd_vel_joy` (geometry_msgs/Twist)
`/cmd_vel_key` (geometry_msgs/Twist)
`/cmd_vel_nav` (geometry_msgs/Twist)
`/cmd_vel_web` (geometry_msgs/Twist)
`/cmd_vel_gesture` (geometry_msgs/Twist)
`/cmd_vel_out` (geometry_msgs/Twist) - Output from twist_mux
`/cmd_vel_stamped` (geometry_msgs/TwistStamped) - Timestamped output
`/emergency_stop` (std_msgs/Bool)


### Nodes
1. **joy_node**
Reads data from joystick hardware

2. **teleop_node**
Converts joystick inputs to Twist messages
Publishes to `/cmd_vel_joy`

3. **teleop_twist_keyboard**
Enables keyboard control
Publishes to `/cmd_vel_key`

4. **twist_mux**
Multiplexes Twist messages from multiple sources
Prioritizes control inputs based on configuration
Publishes to `/cmd_vel_out`

5. **twist_stamper**
Converts Twist to TwistStamped messages
Adds frame_id and timestamp
Publishes to `/cmd_vel_stamped`

6. **safety_stop_node**
Provides emergency stop functionality.
Publishes to `/emergency_stop`.

## Dependencies
This package relies on:

- `joy` - For joystick input
- `teleop_twist_joy` - For joystick teleop
- `teleop_twist_keyboard` - For keyboard teleop
- `twist_mux` - For multiplexing control inputs
- `twist_stamper` - For adding timestamps to Twist messages
- `pynput` - For keyboard input in the `safety_stop_node`.