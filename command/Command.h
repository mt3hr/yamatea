#ifndef Command_H
#define Command_H

#include "RobotAPI.h"

// Command
// CommandExecutorで実行されるコマンドのインターフェース。
// 
// 実方
class Command
{
public:
    virtual ~Command();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual Command *generateReverseCommand();
};
#endif