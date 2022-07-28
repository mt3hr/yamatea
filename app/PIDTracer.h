#ifndef PIDTracer_H
#define PIDTracer_H

#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"

using namespace ev3api;

// PIDTraceMode
// 左ライントレースか、右ライントレースか
//
// 実方
enum PIDTracerMode
{
    LEFT_TRACE,
    RIGHT_TRACE,
};

// PIDTracer
// PIDの値をもとにラインに沿って走行するトレーサ。
//
// 実方
class PIDTracer : public Command
{
private:
    int pwm = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float dt = 0;
    int targetBrightness = 0;
    Motor *leftWheel;
    Motor *rightWheel;
    ColorSensor *colorSensor;
    float beforeP = 0;
    PIDTracerMode mode;

public:
    PIDTracer(PIDTracerMode mode, int pwm, float kp, float ki, float kd, float dt, int target, Motor *leftMotor, Motor *rightMotor, ColorSensor *colorSensor);
    void run() override;
    void setTargetBrightness(int targetBrightness);
};

#endif