#include "ros_interface.h"
// #include "utils.h"
#include "globals.h"

#include <micro_ros_arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <tf2_msgs/msg/tf_message.h>
#include <string.h>
#include <std_msgs/msg/int32.h>
#include <rcutils/logging_macros.h>

// Globale ROS-Variablen
rcl_subscription_t subscriber;
geometry_msgs__msg__TwistStamped twistStampedMsg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t odomPublisher;
nav_msgs__msg__Odometry odomMsg;
rcl_timer_t controlTimer;

// Joint States Publisher
rcl_publisher_t jointStatesPublisher;
sensor_msgs__msg__JointState jointStatesMsg;

// TF Publisher
rcl_publisher_t tfPublisher;
tf2_msgs__msg__TFMessage tfMessage;
geometry_msgs__msg__TransformStamped transformStamped;
rmw_qos_profile_t rmw_qos_profile_static_tf;

// Timing Variablen
unsigned long prevCmdTime = 0;
unsigned long prevOdomUpdate = 0;
unsigned long last_sync_time = 0;
const unsigned long SYNC_TIMEOUT = 500; // Sync alle 0.5 s


// Timing Funktionen
static unsigned long long time_offset = 0;

struct timespec getTime() {
    struct timespec tp;
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// synchronize time between MicroROS and ROS2
bool syncTime() {
    static bool first_sync = true;
    if (first_sync) {
        if (rmw_uros_sync_session(1000) != RCL_RET_OK) return false;
        first_sync = false;
    } else {
        if (rmw_uros_sync_session(10) != RCL_RET_OK) return false;
    }
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    if (ros_time_ms == 0) {
        return false;
    }
    time_offset = ros_time_ms - millis();
    return true;
}

// Callback für /cmd_vel_stamped
void cmdVelStampedCallback(const void* msgin) {
    prevCmdTime = millis();
    const geometry_msgs__msg__TwistStamped* msg = (const geometry_msgs__msg__TwistStamped*) msgin;
    float linearVelocity = msg->twist.linear.x;
    float angularVelocity = msg->twist.angular.z;
    leftWheel.setTargetSpeed(linearVelocity - angularVelocity * WHEELS_Y_DISTANCE / 2.0);
    rightWheel.setTargetSpeed(linearVelocity + angularVelocity * WHEELS_Y_DISTANCE / 2.0);
}

// Timer-Callback für die Motorsteuerung
void motorControlCallback(rcl_timer_t* timer, int64_t last_call_time) {
    unsigned long currentTime = millis();
    prevCmdTime = currentTime;
    
    // Werte aus der zuletzt empfangenen /cmd_vel_stamped Nachricht
    float linearVelocity = twistStampedMsg.twist.linear.x;
    float angularVelocity = twistStampedMsg.twist.angular.z;
    
    // Berechne die gewünschten Geschwindigkeiten für die Räder
    float vL = (linearVelocity - (angularVelocity * 0.5)) * 20;
    float vR = (linearVelocity + (angularVelocity * 0.5)) * 20;
    
    float currentRpmL = leftWheel.getRpm();
    float currentRpmR = rightWheel.getRpm();
    
    float actuatingSignalL = leftWheel.pid(vL, currentRpmL);
    float actuatingSignalR = rightWheel.pid(vR, currentRpmR);
    
    if (currentTime - prevCmdTime > 1000 || (vL == 0 && vR == 0)) {
        leftWheel.stop();
        rightWheel.stop();
        actuatingSignalL = 0;
        actuatingSignalR = 0;
    } else {
        rightWheel.moveBase(actuatingSignalR, PWM_THRESHOLD, PWM_CHANNEL_R);
        leftWheel.moveBase(actuatingSignalL, PWM_THRESHOLD, PWM_CHANNEL_L);
    }
    
    // Odometry-Berechnung
    float average_rps_x = ((currentRpmL + currentRpmR) / 2.0) / 60.0;
    float linear_x = average_rps_x * WHEEL_CIRCUMFERENCE;
    float average_rps_a = ((-currentRpmL + currentRpmR) / 2.0) / 60.0;
    float angular_z = (average_rps_a * WHEEL_CIRCUMFERENCE) / (WHEELS_Y_DISTANCE / 2.0);
    float linear_y = 0;
    unsigned long now = millis();
    float vel_dt = (now - prevOdomUpdate) / 1000.0;
    prevOdomUpdate = now;
    odometry.update(vel_dt, linear_x, linear_y, angular_z);

    publishOdom();
    publishJointStates(currentRpmL, currentRpmR);
}


void publishOdom() {
    odomMsg = odometry.getData();
    
    odomMsg.header.frame_id.data = (char*)"odom";
    odomMsg.child_frame_id.data = (char*)"base_footprint";
    struct timespec ts = getTime();
    odomMsg.header.stamp.sec = ts.tv_sec;
    odomMsg.header.stamp.nanosec = ts.tv_nsec;

    rcl_ret_t ret = rcl_publish(&odomPublisher, &odomMsg, NULL);
        if (ret != RCL_RET_OK) {
        RCUTILS_LOG_ERROR("Fehler beim Publizieren der Odometrie");
    }
}

// Publiziert die Joint States
void publishJointStates(float currentRpmL, float currentRpmR) {
    jointStatesMsg.header.frame_id.data = (char*)"base_footprint";
    
    static rosidl_runtime_c__String jointNames[2];
    jointNames[0].data = (char*)LEFT_WHEEL_JOINT;
    jointNames[1].data = (char*)RIGHT_WHEEL_JOINT;
    jointStatesMsg.name.data = jointNames;
    jointStatesMsg.name.size = 2;
    jointStatesMsg.name.capacity = 2;
    
    static double positions[2];
    positions[0] = leftWheel.getWheelPosition();
    positions[1] = rightWheel.getWheelPosition();
    jointStatesMsg.position.data = positions;
    jointStatesMsg.position.size = 2;
    jointStatesMsg.position.capacity = 2;
    
    static double velocities[2];
    velocities[0] = (currentRpmL * 2.0 * M_PI) / 60.0;
    velocities[1] = (currentRpmR * 2.0 * M_PI) / 60.0;
    jointStatesMsg.velocity.data = velocities;
    jointStatesMsg.velocity.size = 2;
    jointStatesMsg.velocity.capacity = 2;
    
    struct timespec ts = getTime();
    jointStatesMsg.header.stamp.sec = ts.tv_sec;
    jointStatesMsg.header.stamp.nanosec = ts.tv_nsec;
    
    rcl_ret_t ret = rcl_publish(&jointStatesPublisher, &jointStatesMsg, NULL);
    if (ret != RCL_RET_OK) {
        RCUTILS_LOG_ERROR("Fehler beim Publizieren der Joint States");
    }
}

// Publiziert die statische TF-Transformation
void publishTf() {
    tfMessage.transforms.size = 1;
    tfMessage.transforms.data = &transformStamped;
    
    transformStamped.header.frame_id.data = (char*)"base_footprint";
    transformStamped.child_frame_id.data = (char*)"base_link";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0125;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    
    struct timespec ts = getTime();
    transformStamped.header.stamp.sec = ts.tv_sec;
    transformStamped.header.stamp.nanosec = ts.tv_nsec;
    
    rcl_ret_t ret = rcl_publish(&tfPublisher, &tfMessage, NULL);
    if (ret != RCL_RET_OK) {
        RCUTILS_LOG_ERROR("Fehler beim Publizieren der Transformationen");
    }
}

// Interrupt-Handler für den linken Encoder
void updateEncoderL() {
    if (digitalRead(leftWheel.EncoderPinB) > digitalRead(leftWheel.EncoderPinA))
        leftWheel.EncoderCount.data++;
    else
        leftWheel.EncoderCount.data--;
}

// Interrupt-Handler für den rechten Encoder
void updateEncoderR() {
    if (digitalRead(rightWheel.EncoderPinA) > digitalRead(rightWheel.EncoderPinB))
        rightWheel.EncoderCount.data++;
    else
        rightWheel.EncoderCount.data--;
}

// Initialisiert alle ROS-Komponenten
void setupROS() {
    set_microros_serial_transports(Serial);
    set_microros_transports();
    
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support);
    
    // Subscriber für /cmd_vel_stamped
    rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/cmd_vel_stamped");
    
    // Publisher für Odometry
    rclc_publisher_init_default(&odomPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom");
    
    // Timer für die Motorsteuerung
    const unsigned int samplingT = 10;
    rclc_timer_init_default(&controlTimer, &support, RCL_MS_TO_NS(samplingT), motorControlCallback);
    
    // Executor initialisieren und Callbacks hinzufügen
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &twistStampedMsg, cmdVelStampedCallback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &controlTimer);
    
    // Joint States Publisher
    rclc_publisher_init_default(&jointStatesPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states");
    
    // TF Publisher (für /tf_static mit TRANSIENT_LOCAL QoS)
    rclc_publisher_init(&tfPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf_static", &rmw_qos_profile_static_tf);
    
    publishTf();
}

// Führt den ROS-Spin aus (muss in loop() aufgerufen werden)
void spinROS() {
    if (millis() - last_sync_time > SYNC_TIMEOUT) {
        syncTime();
        last_sync_time = millis();
    }
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
