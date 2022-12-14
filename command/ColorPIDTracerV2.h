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
// 曲率項追加済み、I制御修正済み。
// setTargetColorしてから実行してください
//
// 実方
class ColorPIDTracerV2 : public ColorPIDTracer
{
private:
    float pwm = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float dt = 0;
    float r = 0;
    rgb_raw_t targetRGB;
    float beforeP = 0;
    double integral = 0;
    PIDTracerMode traceMode;
    TraceColor traceColor;

    rgb_raw_t rgb;
    float p;
    float i;
    float d;
    float pid;
    float leftPower;
    float rightPower;

public:
    ColorPIDTracerV2(PIDTracerMode traceMode, TraceColor traceColor, float pwm, float kp, float ki, float kd, float dt, float r);
    virtual ~ColorPIDTracerV2();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorPIDTracerV2 *generateReverseCommand() override;

    // TargetColorを設定するメソッド
    virtual void setTargetColor(rgb_raw_t targetRGB) override;
};

#endif