#ifndef PIDTracer_H
#define PIDTracer_H

#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "PrintMessage.h"
#include "WheelController.h"

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
    float beforeP = 0;
    PIDTracerMode traceMode;
    ColorSensor *colorSensor;
    WheelController *wheelController;
    PrintMessage *printMessage; // NOTE モデルには反映しません

public:
    PIDTracer(PIDTracerMode traceMode, int pwm, float kp, float ki, float kd, float dt, int targetBrightness, WheelController *wheelController, ColorSensor *colorSensor);
    ~PIDTracer();
    void run() override;
    PIDTracer *generateReverseCommand() override;
    void setTargetBrightness(int targetBrightness);
};

#endif