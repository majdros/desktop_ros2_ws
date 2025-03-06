#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <micro_ros_arduino.h>
#include <std_msgs/msg/int32.h>
#include <math.h>

class MotorController {
public:

    MotorController(int8_t forwardPin, int8_t backwardPin, int8_t enablePin,
        int8_t encoderA, int8_t encoderB, int tickPerRevolution,
        float startDeadband, float stopDeadband, float voltsToPwm);

    // Motor-Pins
    int8_t Forward;
    int8_t Backward;
    int8_t Enable;
    int8_t EncoderPinA;
    int8_t EncoderPinB;
    
    // Encoder-Variablen
    std_msgs__msg__Int32 EncoderCount;
    volatile long CurrentPosition;
    volatile long PreviousPosition;
    volatile unsigned long CurrentTime;
    volatile unsigned long PreviousTime;
    volatile unsigned long CurrentTimeForError;
    volatile unsigned long PreviousTimeForError;
    
    // PID-Regelung
    float targetSpeed;
    float rpmFilt;
    float ederivative;
    float rpmPrev;
    float kp;
    float ki;
    float kd;
    float error;
    float previousError;

    // Motor-Steuerungswerte
    float VOLTS_TO_PWM;
    int tick;

    // Methoden-Deklaration
    void initPID(float proportionalGain, float integralGain, float derivativeGain);
    void setTargetSpeed(float speed);
    float getRpm();
    float pid(float setpoint, float feedback);
    void moveBase(float actuatingSignal, int threshold, int pwmChannel);
    void stop();
    double getWheelPosition();

private:
    // PID Anti-Windup
    const float MAX_INTEGRAL = 255.0; // Verhindert Integralwindup
    const float MIN_INTEGRAL = -255.0;
    float eintegral = 0.0;
    // Deadband-Werte
    const float START_DEADBAND;
    const float STOP_DEADBAND;
};

#endif // MOTOR_CONTROLLER_H
