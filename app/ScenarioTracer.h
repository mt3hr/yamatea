#ifndef ScenarioTracer_H
#define ScenarioTracer_H

#include "Motor.h"
#include "Command.h"
#include "WheelController.h"

using namespace ev3api;

// ScenarioTracer
// シンプルなシナリオトレーサ
// 左右モータのpwmを設定してそれに従って走行させ続けるもの
//
// 実方
class ScenarioTracer : public Command
{
private:
    int leftPow;
    int rightPow;
    WheelController *wheelController;

public:
    ScenarioTracer(int leftPow, int rightPow, WheelController *wheelController);
    void run() override;
    ScenarioTracer *generateReverseCommand() override;
};

#endif