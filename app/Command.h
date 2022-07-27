#ifndef Command_H
#define Command_H

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
    virtual void run();
};
#endif