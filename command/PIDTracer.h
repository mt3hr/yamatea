#ifndef PIDTracer_H
#define PIDTracer_H

#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

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
// setTargetBrightnessしてから実行してください
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

    int brightness;
    float p;
    float i;
    float d;
    float pid;
    int leftPower;
    int rightPower;

public:
    PIDTracer(PIDTracerMode traceMode, int pwm, float kp, float ki, float kd, float dt);
    virtual ~PIDTracer();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDTracer *generateReverseCommand() override;
    virtual void setTargetBrightness(int targetBrightness);
};

#endif