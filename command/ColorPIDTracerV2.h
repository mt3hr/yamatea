#ifndef ColorPIDTracerV2_H
#define ColorPIDTracerV2_H

#include "PIDTracer.h"
#include "ColorPIDTracer.h"
#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// ColorPIDTracerV2
// PIDの値をもとにラインに沿って走行するトレーサ。
// setTargetColorしてから実行してください
//
// 実方
class ColorPIDTracerV2 : public ColorPIDTracer
{
private:
    int pwm = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float dt = 0;
    float r = 0;
    rgb_raw_t targetRGB;
    float beforeP = 0;
    PIDTracerMode traceMode;
    TraceColor traceColor;

    rgb_raw_t rgb;
    float integral = 0;
    float p;
    float i;
    float d;
    float pid;
    float pidr;
    int leftPower;
    int rightPower;

public:
    ColorPIDTracerV2(PIDTracerMode traceMode, TraceColor traceColor, int pwm, float kp, float ki, float kd, float dt, float r);
    virtual ~ColorPIDTracerV2();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorPIDTracerV2 *generateReverseCommand() override;

    // TargetColorを設定するメソッド
    virtual void setTargetColor(rgb_raw_t targetRGB) override;
};

#endif