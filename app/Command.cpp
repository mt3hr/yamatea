#include "Command.h"

// オーバーライドして使って
void Command::run()
{
    return;
}

// 左右反転したコマンドを生成する。
// オーバーライドして使って。
Command *Command::generateReverseCommand()
{
    return new Command();
}