# Hand Gesture Control ROS2 Package

A ROS2 package that enables robot control through hand gestures using computer vision and MediaPipe hand tracking.


## Overview

This package implements real-time hand gesture recognition to control a robot using:
- OpenCV for image processing
- MediaPipe for hand landmark detection
- ROS2 for robot control integration
- Custom message types for hand landmark data
- Gesture mapping:

   ```
   0 fingers: Forward   (linear.x = 1.1, angular.z  = 0.0)
   1 finger: Forward    (linear.x = 1.1, angular.z  = 0.0)
   2 fingers: backward  (linear.x = -1.1, angular.z = 0.0)
   3 fingers: Left      (linear.x = 0.0, angular.z  = 1.7)
   4 fingers: Right     (linear.x = 0.0, angular.z  = -1.7)
   5 fingers: Stop      (linear.x = 0.0, angular.z  = 0.0)
   ```

<p align="center">
  <img src="images/Demo.gif" alt="Demo showing hand gesture recognition and robot response (Note: The apparent delay is not a system latency but due to the merging of two separate video recordings that are not perfectly time-synchronized)">

  <em>Demo: Hand gesture control in action (the apparent delay is due to video editing, not actual system latency)</em>
</p>



## Usage

1. Launch:

-  Launch with default settings
```bash
ros2 launch hand_gesture_control hand_gesture_control.launch.py
```

- Launch without Visualization but with twist_stamper 
```bash
ros2 launch hand_gesture_control hand_gesture_control.launch.py draw_enabled:=false use_twist_stamper:=true
```

2. Available Launch Parameters:

| Parameter                | Description                                      | Default Value     |
|--------------------------|--------------------------------------------------|-------------------|
| `use_twist_stamper`      | Enables/disables the twist_stamper node         | `false`            |
| `twist_stamper_frame_id` | Frame ID for TwistStamped messages              | `''` (empty string) |
| `draw_enabled`           | Enables/disables hand tracking visualization    | `true`            |

3. Run individual nodes:
```bash
ros2 run hand_gesture_control hand_tracking_node
ros2 run hand_gesture_control gesture_control_node
ros2 run hand_gesture_control twist_stamper_node
```

## Velocity Command Flexibility

This project supports two formats for speed commands::

1. **Standard Twist Messages** (`geometry_msgs/Twist`)
   - Topic: `/cmd_vel_gesture`
   - Used for robot control without time stamping
   - Issued by `gesture_control_node`

2. **Timestamped Twist Messages** (`geometry_msgs/TwistStamped`)
   - Topic: `/cmd_vel_stamped`
   - Includes additional timestamp information for more precise motion control
   - Issued by `twist_stamper_node`
   - Required for integration with the MicroROS-Bot

The choice between these formats depends on the configuration and requirements of the controlled robot. The MicroROS-Bot uses TwistStamped messages by default for more precise timing control and synchronization.

You can disable the `twist_stamper_node` using the `use_twist_stamper` launch argument


## Package Structure

```bash
hand_gesture_control/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── scripts/
│   ├── hand_tracking_node.py
│   ├── gesture_control_node.py
│   ├── twist_stamper.py
│   └── HandTrackingModule.py
├── msg/
│   ├── HandLandmark.msg
│   └── HandLandmarks.msg
├──  launch/
│    └── gesture_control.launch.py
└── images/
    └── rosgraph.png
```
## Components

![ROS2 Topic Graph](images/rosgraph.png)

### Topics

- `/hand_landmarks` (hand_gesture_control/HandLandmarks)
- `/cmd_vel_gesture` (geometry_msgs/Twist)
- `/cmd_vel_stamped` (geometry_msgs/TwistStamped)

### Nodes

1. **hand_tracking_node**
   - Subscribes to `/image_raw`
   - Uses MediaPipe for hand landmark detection
   - Publishes hand landmarks to `/hand_landmarks`
   - Parameters:
     - `draw_enabled` (bool, default: True): Enable visualization
     - `max_hands` (int, default: 1): Maximum number of hands to detect
     - `detection_confidence` (float, default: 0.9)
     - `tracking_confidence` (float, default: 0.9)

2. **gesture_control_node**
   - Subscribes to `/hand_landmarks`
   - Converts hand gestures to robot commands
   - Publishes Twist messages to `/cmd_vel_gesture`
   - Provides the `Gesture mapping`

3. **twist_stamper_node**
   - Converts Twist to TwistStamped messages
   - Publishes TwistStamped messages to `/cmd_vel_stamped`
   - Parameters:
     - `twist_stamper_frame_id` (string, default: ''): Frame ID for TwistStamped messages

### Custom Messages

#### HandLandmark.msg
```
uint8 id      # Landmark identifier (0-20)
int32 x       # x coordinate in pixels
int32 y       # y coordinate in pixels
```

#### HandLandmarks.msg
```
HandLandmark[] landmarks  # Array of hand landmarks
```
