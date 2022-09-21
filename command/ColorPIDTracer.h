#ifndef ColorPIDTracer_H
#define ColorPIDTracer_H

#include "PIDTracer.h"
#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// TraceColor
// ColorPIDTracerでトレースする色
//
// 実方
enum TraceColor
{
    Trace_R, // 赤
    Trace_G, // 緑
    Trace_B, // 青
};

// ColorPIDTracer
// PIDの値をもとにラインに沿って走行するトレーサ。
// setTargetColorしてから実行してください
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
    rgb_raw_t targetRGB;
    float beforeP = 0;
    PIDTracerMode traceMode;
    TraceColor traceColor;

    rgb_raw_t rgb;
    float p;
    float i;
    float d;
    float pid;
    int leftPower;
    int rightPower;

public:
    ColorPIDTracer(PIDTracerMode traceMode, TraceColor traceColor, int pwm, float kp, float ki, float kd, float dt);
    virtual ~ColorPIDTracer();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorPIDTracer *generateReverseCommand() override;

    // TargetColorを設定するメソッド
    virtual void setTargetColor(rgb_raw_t targetRGB);
};

#endif