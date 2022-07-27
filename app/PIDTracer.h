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
    PIDTracer(PIDTracerMode modea, int pwma, float kpa, float kia, float kda, float dta, int targeta, Motor *leftMotora, Motor *rightMotora, ColorSensor *colorSensora);
    void run() override;
    void setTargetBrightness(int targetBrightness);
};

#endif