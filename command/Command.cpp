#include "Command.h"
#include "RobotAPI.h"

Command::~Command()
{
}

// オーバーライドして使って
void Command::run(RobotAPI *robotAPI)
{
    return;
}

// 左右反転したコマンドを生成する。
// オーバーライドして使って。
Command *Command::generateReverseCommand()
{
    return new Command();
}