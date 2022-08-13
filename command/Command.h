#ifndef Command_H
#define Command_H

#include "RobotAPI.h"

// Command
// CommandExecutorで実行されるコマンドのインターフェース。
// オーバーライドして使ってください。
// 既知のCommand:Command
//               PIDTargetBrightnessCalibrator
//               PIDTracer
//               ScenarioTracer
// 実方
class Command
{
public:
    virtual ~Command();
    virtual void run(RobotAPI *robotAPI);
    virtual Command *generateReverseCommand();
};
#endif