#ifndef ColorPIDTracer_H
#define ColorPIDTracer_H

#include "PIDTracer.h"
#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// PIDTracer
// PIDの値をもとにラインに沿って走行するトレーサ。
//
// 実方
class ColorPIDTracer : public Command
{
private:
    int pwm = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float dt = 0;
    int targetRed = 0;
    float beforeP = 0;
    PIDTracerMode traceMode;

    int red;
    float p;
    float i;
    float d;
    float pid;
    int leftPower;
    int rightPower;

public:
    ColorPIDTracer(PIDTracerMode traceMode, int pwm, float kp, float ki, float kd, float dt);
    virtual ~ColorPIDTracer();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorPIDTracer *generateReverseCommand() override;
    virtual void setTargetRed(int targetRed);
};

#endif