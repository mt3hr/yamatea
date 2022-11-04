#ifndef PIDTracerV3_H
#define PIDTracerV3_H

#include "PIDTracer.h"
#include "Motor.h"
#include "Command.h"
#include "ColorSensor.h"
#include "RobotAPI.h"

using namespace ev3api;
using namespace std;

// PIDTracerV3
// PIDの値をもとにラインに沿って走行するトレーサ。
// 曲率項追加済み、I制御修正済み。
// setTargetBrightnessしてから実行してください
//
// 実方
class PIDTracerV3 : public PIDTracer
{
private:
    float pwm = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float dt = 0;
    float r = 0;
    int8_t targetBrightness = 0;
    float beforeP = 0;
    static const double *integral;
    PIDTracerMode traceMode;

    int8_t brightness;
    float p;
    float i;
    float d;
    float pid;
    float leftPower;
    float rightPower;

public:
    PIDTracerV3(PIDTracerMode traceMode, float pwm, float kp, float ki, float kd, float dt, float r);
    virtual ~PIDTracerV3();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual PIDTracerV3 *generateReverseCommand() override;

    // TargetBrightnessを設定するメソッド
    virtual void setTargetBrightness(int8_t targetBrightness) override;
};
#endif