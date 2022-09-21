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

    // コマンドを実行する。オーバーライドして使って
    virtual void run(RobotAPI *robotAPI);

    // 事前準備処理。Commandが呼び出される直前に一度だけ呼び出される。オーバーライドして使って
    virtual void preparation(RobotAPI *robotAPI);

    // Commandを左右反転する。オーバーライドして使って
    virtual Command *generateReverseCommand();
};
#endif