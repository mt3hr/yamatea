#ifndef BatteryEaterSilent_H
#define BatteryEaterSilent_H

#include "RobotAPI.h"
#include "Command.h"

// BatteryEaterSilent
// BatteryEaterSilentExecutorで実行されるコマンドのインターフェース。
// 
// 実方
class BatteryEaterSilent : public Command
{
public:
    virtual ~BatteryEaterSilent();

    virtual void run(RobotAPI *robotAPI) override;

    virtual void preparation(RobotAPI *robotAPI) override;

    virtual BatteryEaterSilent *generateReverseCommand() override;
};
#endif