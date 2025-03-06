#include <micro_ros_arduino.h>
#include "ros_interface.h"
#include "MotorController.h"
#include "globals.h"

Odometry odometry;

// Linksrad
const int8_t L_FORW         = 27;
const int8_t L_BACK         = 26;
const int8_t L_ENABLE       = 25;
const int8_t L_ENCODER_PIN1 = 21;
const int8_t L_ENCODER_PIN2 = 18;

// Rechtsrad
const int8_t R_FORW         = 32;
const int8_t R_BACK         = 33;
const int8_t R_ENABLE       = 5;
const int8_t R_ENCODER_PIN1 = 15;
const int8_t R_ENCODER_PIN2 = 23;

// Roboter Parameter
const float WHEELS_Y_DISTANCE   = 0.155;
const float WHEEL_RADIUS        = 0.033;
const float WHEEL_CIRCUMFERENCE = 2 * 3.14 * WHEEL_RADIUS;

// Encoder: Ticks pro Umdrehung
const int TICKS_PER_REV_LW = 960;
const int TICKS_PER_REV_RW = 960;

// PID-Konstanten
const float KP_L = 6.0, KI_L = 5.5, KD_L = 0.0;
const float KP_R = 6.0, KI_R = 5.5, KD_R = 0.0;

// PWM Parameter
const int PWM_FREQ          = 20000;      // 20 kHz
const int PWM_CHANNEL_L     = 0;
const int PWM_CHANNEL_R     = 1;
const int PWM_RESOLUTION    = 8;          // 8-Bit (0-255)
const int PWM_THRESHOLD     = 0;
float VOLTS_TO_PWM          = 255.0f / 6.0f; // 42.5 PWM-Schritte pro Volt
const float START_DEADBAND_L= 0.735 * 1.1;
const float START_DEADBAND_R= 0.685 * 1.1;
const float STOPDEADBAND_L  = 0.62 * 1.1;
const float STOPDEADBAND_R  = 0.585 * 1.1; 

// Joint-Namen
const char* LEFT_WHEEL_JOINT  = "base_left_wheel_joint";
const char* RIGHT_WHEEL_JOINT = "base_right_wheel_joint";

// Definition globaler Objekte
MotorController leftWheel(L_FORW, L_BACK, L_ENABLE, L_ENCODER_PIN1, L_ENCODER_PIN2, TICKS_PER_REV_LW, START_DEADBAND_L, STOPDEADBAND_L, VOLTS_TO_PWM);
MotorController rightWheel(R_FORW, R_BACK, R_ENABLE, R_ENCODER_PIN1, R_ENCODER_PIN2, TICKS_PER_REV_RW, START_DEADBAND_R, STOPDEADBAND_R, VOLTS_TO_PWM);



void setup() {
    Serial.begin(115200);

    // PID-Initialisierung
    leftWheel.initPID(KP_L, KI_L, KD_L);
    rightWheel.initPID(KP_R, KI_R, KD_R);

    // Encoder-Interrupts konfigurieren
    attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB), updateEncoderL, RISING);
    attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinA), updateEncoderR, RISING);
    
    // PWM-Konfiguration
    ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(leftWheel.Enable, PWM_CHANNEL_L);
    ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(rightWheel.Enable, PWM_CHANNEL_R);
    
    // ROS initialisieren
    setupROS();
}

void loop() {
    spinROS();
}