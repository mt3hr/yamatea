#ifndef ExecutePreparationWhenExitBeforeCommandHandler_H
#define ExecutePreparationWhenExitBeforeCommandHandler_H

#include "Preparizable.h"
#include "Handler.h"

// ExecutePreparationWhenExitBeforeCommandHandler
// 開始のタイミングで準備処理が必要なコマンドに、準備メソッド呼び出しを実現するクラス。
//
// 車体を指定角度旋回させたい時やアームを指定角度回転させたい時などに
// 一つ前のCommandが終わったときに呼び出されるようにする。
// 一つ前のCommandをCommandExecutor.addCommand()にわたすとき、
// 同時にExecutePreparationWhenExitBeforeCommandHandlerも渡すことで呼び出しを実現する。
//
// 実方
class ExecutePreparationWhenExitBeforeCommandHandler : public Handler
{
private:
    Preparizable *preparizable;

public:
    ExecutePreparationWhenExitBeforeCommandHandler(Preparizable *preparizable);
    void handle() override;
};

#endif