#ifndef GLOBALS_H
#define GLOBALS_H

#include "MotorController.h"
#include "odometry.h"


// Globale MotorController-Objekte
extern MotorController leftWheel;
extern MotorController rightWheel;

// Globales Odometry-Objekt
extern Odometry odometry;

// Roboterparameter
extern const float WHEELS_Y_DISTANCE;
extern const float WHEEL_CIRCUMFERENCE;

// PWM Parameter
extern const int PWM_THRESHOLD;
extern const int PWM_CHANNEL_R;
extern const int PWM_CHANNEL_L;

// Joint-Namen
extern const char* LEFT_WHEEL_JOINT;
extern const char* RIGHT_WHEEL_JOINT;

#endif // GLOBALS_H
