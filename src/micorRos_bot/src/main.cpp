#include <micro_ros_arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rcutils/logging_macros.h>
#include <odometry.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw/qos_profiles.h>


//pin declaration
//Left wheel
int8_t L_FORW = 27;
int8_t L_BACK = 26;
int8_t L_enablePin = 25;
int8_t L_encoderPin1 = 21;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t L_encoderPin2 = 18;
//right wheel
int8_t R_FORW = 32;
int8_t R_BACK = 33;
int8_t R_enablePin = 5;
int8_t R_encoderPin1 = 15;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t R_encoderPin2 = 23;

// funktions declaration
void updateEncoderL();
void updateEncoderR();
void MotorControll_callback(rcl_timer_t* timer, int64_t last_call_time);
void cmd_vel_stamped_callback(const void* msgin);
void publishOdom();
void publishTf();
void publishJointStates(float currentRpmL, float currentRpmR);
struct timespec getTime();
bool syncTime();


//parameters of the robot
float wheels_y_distance_ = 0.155;
float wheel_radius = 0.033;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;

//encoder value per revolution of left wheel and right wheel
int tickPerRevolution_LW = 960;
int tickPerRevolution_RW = 960;
int threshold = 0;

//pid constants of left wheel
float kp_l = 6.0, ki_l = 5.5, kd_l = 0.0;
//pid constants of right wheel
float kp_r = 6.0, ki_r = 5.5, kd_r = 0.0;

//pwm parameters setup
const int freq = 20000;      // freq 20 kHz
const int pwmChannelL = 0;
const int pwmChannelR = 1;
const int resolution = 8;    //8-Bit resolutin (0-255)
float volts_to_pwm = 255.0f / 6.0f; // 42.5 PWM-Schritte pro Volt
float startDeadbandL = 0.735 * 1.1;
float startDeadbandR = 0.685 * 1.1;
float stopDeadbandL = 0.62 * 1.1;
float stopDeadbandR = 0.585 * 1.1; 

// Joint Names als String-Arrays
const char* left_wheel_joint_name = "base_left_wheel_joint";
const char* right_wheel_joint_name = "base_right_wheel_joint";


rcl_subscription_t subscriber;
geometry_msgs__msg__TwistStamped msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t odom_publisher;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
nav_msgs__msg__Odometry odom_msg;
rcl_timer_t timer;
rcl_timer_t ControlTimer;

// Globale Variablen für Joint States
rcl_publisher_t joint_states_publisher;
sensor_msgs__msg__JointState joint_states_msg;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

// Globale Variablen für TF
rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage tf_message;
geometry_msgs__msg__TransformStamped transform_stamped;
rmw_qos_profile_t rmw_qos_profile_static_tf;

// Globale Variable für den letzten Sync-Zeitpunkt
unsigned long last_sync_time = 0;
const unsigned long SYNC_TIMEOUT = 500; // Sync alle 0.5 Sekunden



//creating a class for motor control
class MotorController {
public:
  int8_t Forward;
  int8_t Backward;
  int8_t Enable;
  int8_t EncoderPinA;
  int8_t EncoderPinB;
  std_msgs__msg__Int32 EncoderCount;
  float targetSpeed;
  volatile long CurrentPosition;
  volatile long PreviousPosition;
  volatile long CurrentTime;
  volatile long PreviousTime;
  volatile long CurrentTimeforError;
  volatile long PreviousTimeForError;
  float rpmFilt = 0.0;
  // float eintegral;
  float ederivative;
  float rpmPrev;
  float kp;
  float ki;
  float kd;
  float error;
  float previousError = 0;
  int tick;

  // Funktion zur Berechnung der Radposition in Radianten
  double getWheelPosition() {
      return (2.0 * M_PI * (double)EncoderCount.data) / (double)tick;
  }

  MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EnablePin,
    int8_t EncoderA, int8_t EncoderB, int tickPerRevolution,
    float startDeadband, float stopDeadband): START_DEADBAND(startDeadband * volts_to_pwm), STOP_DEADBAND(stopDeadband * volts_to_pwm)
    {
    this->Forward = ForwardPin;
    this->Backward = BackwardPin;
    this->Enable = EnablePin;
    this->EncoderPinA = EncoderA;
    this->EncoderPinB = EncoderB;
    this->tick = tickPerRevolution;
    pinMode(Forward, OUTPUT);
    pinMode(Backward, OUTPUT);
    pinMode(EnablePin, OUTPUT);
    pinMode(EncoderPinA, INPUT);
    pinMode(EncoderPinB, INPUT);
  }

  //initializing the parameters of PID controller
  void initPID(float proportionalGain, float integralGain, float derivativeGain) {
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
  }

  // Funktion zum Setzen der Zielgeschwindigkeit
  void setTargetSpeed(float speed) {
    targetSpeed = speed;
  }
  
  //function return rpm of the motor using the encoder tick values
  float getRpm() {
    CurrentPosition = EncoderCount.data;
    CurrentTime = millis();
    float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;

    // Vermeiden von Division durch sehr kleine Zeitdifferenzen
    if (delta1 < 0.001) return rpmFilt;
    
    float velocity = ((float)CurrentPosition - PreviousPosition) / delta1;
    float rpm = (velocity / tick) * 60;
  
    // Verbesserte Filterung mit variabler Filterkonstante
    const float alpha = 0.1;  // Filterkonstante (0.1 = starke Filterung, 0.9 = schwache Filterung)
    rpmFilt = (1 - alpha) * rpmFilt + alpha * rpm;

    PreviousPosition = CurrentPosition;
    PreviousTime = CurrentTime;
    return rpmFilt;
  }

  //pid controller
private:
  const float MAX_INTEGRAL = 255.0; // Maximum integral value to prevent windup
  const float MIN_INTEGRAL = -255.0;
  float eintegral = 0.0;
  const float START_DEADBAND;
  const float STOP_DEADBAND;

public:
  float pid(float setpoint, float feedback) {
    CurrentTimeforError = millis();
    float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
    error = setpoint - feedback;
    
    // Anti-windup: Only integrate if output is not saturated
    float temp_integral = eintegral + (error * delta2);
    if (temp_integral <= MAX_INTEGRAL && temp_integral >= MIN_INTEGRAL) {
      eintegral = temp_integral;
    }
    
    ederivative = (error - previousError) / delta2;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

    // Limit control signal
    if (control_signal > MAX_INTEGRAL) control_signal = MAX_INTEGRAL;
    if (control_signal < MIN_INTEGRAL) control_signal = MIN_INTEGRAL;

    previousError = error;
    PreviousTimeForError = CurrentTimeforError;
    return control_signal;
  }

  //move the robot wheels based the control signal generated by the pid controller
  void moveBase(float ActuatingSignal, int threshold, int pwmChannel) {
    // Deadband compensation
    if (fabs(ActuatingSignal) < STOP_DEADBAND) {
        ActuatingSignal = 0;}
    else if (fabs(ActuatingSignal) < START_DEADBAND){
        if (ActuatingSignal > 0)
          ActuatingSignal = START_DEADBAND;
        else
          ActuatingSignal = -START_DEADBAND;
      }

    if (ActuatingSignal > 0) {
        digitalWrite(Forward, HIGH);
        digitalWrite(Backward, LOW);
    } else {
        digitalWrite(Forward, LOW);
        digitalWrite(Backward, HIGH);
    }

    int pwm = threshold + (int)fabs(ActuatingSignal);
    if (pwm > 255)
        pwm = 255;

    ledcWrite(pwmChannel, pwm);
  }

  void stop() {
    digitalWrite(Forward, LOW);
    digitalWrite(Backward, LOW);
  }
};



//creating objects for right wheel and left wheel
MotorController leftWheel(L_FORW, L_BACK, L_enablePin, L_encoderPin1, L_encoderPin2, tickPerRevolution_LW, startDeadbandL, stopDeadbandL);
MotorController rightWheel(R_FORW, R_BACK, R_enablePin, R_encoderPin1, R_encoderPin2, tickPerRevolution_RW, startDeadbandR, stopDeadbandR);

// #define LED_PIN 2
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { /*error_loop();*/ } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { /*error_loop();*/ } \
  }



// void error_loop() {
//   while (1) {
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//     delay(100);
//   }
// }

void setup() {

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // syncTime();
  //initializing the pid constants
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);
  //initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinA), updateEncoderR, RISING);
  //initializing pwm signal parameters
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(leftWheel.Enable, pwmChannelL);
  ledcSetup(pwmChannelR, freq, resolution);
  ledcAttachPin(rightWheel.Enable, pwmChannelR);

  set_microros_transports();
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);

  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create subscriber for /cmd_vel_stamped topic
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/cmd_vel_stamped"));

  //create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom"));

  //timer function for controlling the motor base. At every samplingT time
  //MotorControll_callback function is called
  //Here I had set SamplingT=10 Which means at every 10 milliseconds MotorControll_callback function is called

  const unsigned int samplingT = 20;
  RCCHECK(rclc_timer_init_default(
    &ControlTimer, &support, RCL_MS_TO_NS(samplingT), MotorControll_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &msg, &cmd_vel_stamped_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));
  // Joint States Publisher initialisieren
  RCCHECK(rclc_publisher_init_default(
    &joint_states_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),"/joint_states"));
  // TF Publisher to /tf_static Topic with "TRANSIENT_LOCAL" QoS
  RCCHECK(rclc_publisher_init(
    &tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf_static", &rmw_qos_profile_static_tf));

  publishTf();
}



void loop() {
  // delay(100);
  if (millis() - last_sync_time > SYNC_TIMEOUT) {
  syncTime();
  last_sync_time = millis();
  }
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}



void cmd_vel_stamped_callback(const void* msgin) {
  prev_cmd_time = millis();

  // Converting the received message into the TwistStamped message format
  const geometry_msgs__msg__TwistStamped* msg = (const geometry_msgs__msg__TwistStamped*)msgin;

  // Linear- und Winkelgeschwindigkeiten aus der Nachricht TwistStamped
  float linearVelocity = msg->twist.linear.x;
  float angularVelocity = msg->twist.angular.z;

  // Speichere die Geschwindigkeit für die Steuerung in der Hauptfunktion
  leftWheel.setTargetSpeed(linearVelocity - angularVelocity * wheels_y_distance_ / 2);
  rightWheel.setTargetSpeed(linearVelocity + angularVelocity * wheels_y_distance_ / 2);
}



//function which controlles the motor
void MotorControll_callback(rcl_timer_t* timer, int64_t last_call_time) {
  // unsigned long currentTime = millis();
  // prev_cmd_time = currentTime;

  float linearVelocity;
  float angularVelocity;
  //linear velocity and angular velocity send cmd_vel_stamped topic
  linearVelocity = msg.twist.linear.x;
  angularVelocity = msg.twist.angular.z;
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
  float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;
  //current wheel rpm is calculated
  float currentRpmL = leftWheel.getRpm();
  float currentRpmR = rightWheel.getRpm();
  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = leftWheel.pid(vL, currentRpmL);
  float actuating_signal_RW = rightWheel.pid(vR, currentRpmR);
  if (vL == 0 && vR == 0) {
    leftWheel.stop();
    rightWheel.stop();
    actuating_signal_LW = 0;
    actuating_signal_RW = 0;
  } 
  else {
    rightWheel.moveBase(actuating_signal_RW, threshold, pwmChannelR);
    leftWheel.moveBase(actuating_signal_LW, threshold, pwmChannelL);
  }
  //odometry
  float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0;  // RPM
  float linear_x = average_rps_x * wheel_circumference_;                  // m/s
  float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
  float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0);  //  rad/s
  float linear_y = 0;
  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(vel_dt, linear_x, linear_y, angular_z);
  publishOdom();
  publishJointStates(currentRpmL, currentRpmR);
  // publishTf();
}



//interrupt function for left wheel encoder.
void updateEncoderL() {
  if (digitalRead(leftWheel.EncoderPinB) > digitalRead(leftWheel.EncoderPinA))
    leftWheel.EncoderCount.data++;
  else
    leftWheel.EncoderCount.data--;
  encodervalue_l = leftWheel.EncoderCount;
}


//interrupt function for right wheel encoder
void updateEncoderR() {
  if (digitalRead(rightWheel.EncoderPinA) > digitalRead(rightWheel.EncoderPinB))
    rightWheel.EncoderCount.data++;
  else
    rightWheel.EncoderCount.data--;
  encodervalue_r = rightWheel.EncoderCount;
}



//function which publishes joint states in Topic /joint_states
void publishJointStates(float currentRpmL, float currentRpmR){

  // initialize Joint States Message
  joint_states_msg.header.frame_id.data = (char*)"base_footprint";
  joint_states_msg.header.frame_id.size = strlen("base_footprint");
  joint_states_msg.header.frame_id.capacity = strlen("base_footprint");

  // allocate Arrays for Joint States
  joint_states_msg.name.capacity = 2;
  joint_states_msg.name.size = 2;
  joint_states_msg.name.data = (rosidl_runtime_c__String*)malloc(
      joint_states_msg.name.capacity * sizeof(rosidl_runtime_c__String));

  joint_states_msg.position.capacity = 2;
  joint_states_msg.position.size = 2;
  joint_states_msg.position.data = (double*)malloc(
      joint_states_msg.position.capacity * sizeof(double));

  joint_states_msg.velocity.capacity = 2;
  joint_states_msg.velocity.size = 2;
  joint_states_msg.velocity.data = (double*)malloc(
      joint_states_msg.velocity.capacity * sizeof(double));

  // set Joint Names
  joint_states_msg.name.data[0].data = (char*)left_wheel_joint_name;
  joint_states_msg.name.data[0].size = strlen(left_wheel_joint_name);
  joint_states_msg.name.data[0].capacity = strlen(left_wheel_joint_name);

  joint_states_msg.name.data[1].data = (char*)right_wheel_joint_name;
  joint_states_msg.name.data[1].size = strlen(right_wheel_joint_name);
  joint_states_msg.name.data[1].capacity = strlen(right_wheel_joint_name);

  // calculate and set positions of the wheels in rad/s
  joint_states_msg.position.data[0] = leftWheel.getWheelPosition();
  joint_states_msg.position.data[1] = rightWheel.getWheelPosition();

  // Calculate and set the speeds of the wheels in rad/s
  joint_states_msg.velocity.data[0] = (currentRpmL * 2.0 * M_PI) / 60.0;  // conversion from RPM to rad/s
  joint_states_msg.velocity.data[1] = (currentRpmR * 2.0 * M_PI) / 60.0;  // conversion from RPM to rad/s

  // synchronize Joint States Message
  struct timespec time_stamp = getTime();
  joint_states_msg.header.stamp.sec = time_stamp.tv_sec;
  joint_states_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  // publish Joint States
  RCSOFTCHECK(rcl_publish(&joint_states_publisher, &joint_states_msg, NULL));   //  Joint States publizieren
  // rcl_ret_t ret = rcl_publish(&joint_states_publisher, &joint_states_msg, NULL);    //  Joint States publizieren
  // if (ret != RCL_RET_OK) {
  //     RCUTILS_LOG_ERROR("Fehler beim Publizieren der Joint States");
  // }
}



//function which publishes wheel odometry in Topic /wheel_odom
void publishOdom() {
  odom_msg = odometry.getData();

  // set odom-Frame-IDs
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = strlen("odom");
  
  odom_msg.child_frame_id.data = (char*)"base_footprint";
  odom_msg.child_frame_id.size = strlen("base_footprint");
  odom_msg.child_frame_id.capacity = strlen("base_footprint");

  struct timespec time_stamp = getTime();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));      // Odometrie publizieren
  // rcl_ret_t ret = rcl_publish(&odom_publisher, &odom_msg, NULL);      // Odometrie publizieren
  // if (ret != RCL_RET_OK) {
  //     RCUTILS_LOG_ERROR("Fehler beim Publizieren der Odometrie");
  // }
}



//function which publishes static-tf in Topic /tf_static
void publishTf(){
  tf_message.transforms.capacity = 1;
  tf_message.transforms.size = 1;
  tf_message.transforms.data = &transform_stamped;

  // set TF-Frame-IDs
  transform_stamped.header.frame_id.data = (char*)"base_footprint";
  transform_stamped.header.frame_id.size = strlen("base_footprint");
  transform_stamped.header.frame_id.capacity = strlen("base_footprint");

  transform_stamped.child_frame_id.data = (char*)"base_link";
  transform_stamped.child_frame_id.size = strlen("base_link");
  transform_stamped.child_frame_id.capacity = strlen("base_link");

  // set transform from odometry
  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0125;
  
  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = 0.0;
  transform_stamped.transform.rotation.w = 1.0;

  // synchronize transform_stamped Message
  struct timespec time_stamp = getTime();
  transform_stamped.header.stamp.sec = time_stamp.tv_sec;
  transform_stamped.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_message, NULL));       // TF publizieren
  // rcl_ret_t ret = rcl_publish(&tf_publisher, &tf_message, NULL);       // TF publizieren
  // if (ret != RCL_RET_OK) {
  //     RCUTILS_LOG_ERROR("Fehler beim Publizieren der Transformationen");
  // }
}



// synchronize time between MicroROS and ROS2
bool syncTime() {

  static bool first_sync = true;
  if (first_sync) {
    RCCHECK(rmw_uros_sync_session(1000));
    first_sync = false;
  } 
  else {
    // Regular sync with shorter timeout
    RCCHECK(rmw_uros_sync_session(10));
  }
  
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  if (ros_time_ms == 0) {
    return false; // Sync failed
  }
  
  // Update time offset
  time_offset = ros_time_ms - millis();
  return true;
}



struct timespec getTime() {
  struct timespec tp = { 0 };
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}