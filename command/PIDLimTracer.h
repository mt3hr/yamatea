#ifndef PIDLimTracer_H
#define PIDLimTracer_H

#include "PIDTracer.h"
#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

enum PIDLimTracerMode
{
    PLTM_P,
    PLTM_PI,
    PLTM_PID,
    PLTM_PID_MOD,
};

// PIDLimTracer
// 限界感度法を用いたPIDTracer
// setTargetBrightnessしてから実行してください
//
// 実方
class PIDLimTracer : public PIDTracer
{
private:
    PIDLimTracerMode mode;
    float pwm = 0;
    float ku = 0;
    float kp = 0;
    float tu = 0;
    float ti = 0;
    float td = 0;
    float dt = 0;
    float r = 0;
    int8_t targetBrightness = 0;
    float beforeP = 0;
    double integral = 0;
    PIDTracerMode traceMode;

    int8_t brightness;
    float p;
    float i;
    float d;
    float pid;
    float leftPower;
    float rightPower;

public:
    PIDLimTracer(PIDTracerMode traceMode, PIDLimTracerMode mode, float pwm, float ku, float tu, float dt, float r);
    virtual ~PIDLimTracer();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDLimTracer *generateReverseCommand() override;

    // TargetBrightnessを設定するメソッド
    virtual void setTargetBrightness(int8_t targetBrightness) override;
};

#endif