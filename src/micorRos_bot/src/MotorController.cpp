#include "MotorController.h"

MotorController::MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EnablePin,
                                int8_t EncoderA, int8_t EncoderB, int tickPerRevolution,
                                float startDeadband, float stopDeadband, float voltsToPwm)
    :   VOLTS_TO_PWM(voltsToPwm),
        START_DEADBAND(startDeadband * voltsToPwm), 
        STOP_DEADBAND(stopDeadband * voltsToPwm) 
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


void MotorController::initPID(float proportionalGain, float integralGain, float derivativeGain) {
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
}


void MotorController::setTargetSpeed(float speed) {
    targetSpeed = speed;
}


float MotorController::getRpm() {
    CurrentPosition = EncoderCount.data;
    CurrentTime = millis();
    float deltaTime = ((float)CurrentTime - PreviousTime) / 1000.0;

    if (deltaTime < 0.001) return rpmFilt;

    float velocity = ((float)CurrentPosition - PreviousPosition) / deltaTime;
    float rpm = (velocity / tick) * 60.0;
    rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
    rpmPrev = rpm;
    PreviousPosition = CurrentPosition;
    PreviousTime = CurrentTime;
    return rpmFilt;
}


float MotorController::pid(float setpoint, float feedback) {
    CurrentTimeForError = millis();
    float deltaTime = ((float)CurrentTimeForError - PreviousTimeForError) / 1000.0;
    error = setpoint - feedback;
    eintegral += error * deltaTime;
    ederivative = (error - previousError) / deltaTime;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);
    previousError = error;
    PreviousTimeForError = CurrentTimeForError;
    return control_signal;
}


void MotorController::moveBase(float ActuatingSignal, int threshold, int pwmChannel) {
    // Deadband-Kompensation
    if (fabs(ActuatingSignal) < STOP_DEADBAND) {
        ActuatingSignal = 0;}
    else if (fabs(ActuatingSignal) < START_DEADBAND){
        ActuatingSignal = (ActuatingSignal > 0) ? START_DEADBAND : -START_DEADBAND;
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


void MotorController::stop() {
    digitalWrite(Forward, LOW);
    digitalWrite(Backward, LOW);
}


double MotorController::getWheelPosition() {
    return (2.0 * M_PI * (double)EncoderCount.data) / (double)tick;
}
