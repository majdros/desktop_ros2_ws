#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

// #include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <tf2_msgs/msg/tf_message.h>


// Funktionen für ROS-Initialisierung und -Spin
void setupROS();
void spinROS();

// Callback-Funktionen
void cmdVelStampedCallback(const void* msgin);
void motorControlCallback(rcl_timer_t* timer, int64_t last_call_time);

// Publisher
void publishOdom();
void publishJointStates(float currentRpmL, float currentRpmR);
void publishTf();

// Interrupt-Handler für Encoder
void updateEncoderL();
void updateEncoderR();

// Syncronisiere die Zeit mit ROS
struct timespec getTime();
bool syncTime();

#endif // ROS_INTERFACE_H
