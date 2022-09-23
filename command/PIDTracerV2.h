#ifndef PIDTracerV2_H
#define PIDTracerV2_H

#include "PIDTracer.h"
#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// PIDTracerV2
// PIDの値をもとにラインに沿って走行するトレーサ。
// setTargetBrightnessしてから実行してください
//
// 実方
class PIDTracerV2 : public PIDTracer
{
private:
    int pwm = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float dt = 0;
    float r = 0;
    int8_t targetBrightness = 0;
    float beforeP = 0;
    PIDTracerMode traceMode;

    int8_t brightness;
    float p;
    float i;
    float d;
    float pid;
    float pidr;
    int leftPower;
    int rightPower;

public:
    PIDTracerV2(PIDTracerMode traceMode, int pwm, float kp, float ki, float kd, float dt, float r);
    virtual ~PIDTracerV2();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDTracerV2 *generateReverseCommand() override;

    // TargetBrightnessを設定するメソッド
    virtual void setTargetBrightness(int8_t targetBrightness) override;
};

#endif