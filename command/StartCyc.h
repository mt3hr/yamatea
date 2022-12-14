#ifndef StartCyc_H
#define StartCyc_H

#include "Command.h"
#include "RobotAPI.h"
#include "ev3api.h"

using namespace ev3api;

// StartCyc
// 周期ハンドラを起動するコマンド
//
// 実方
class StartCyc : public Command
{
private:
    ID taskName;

public:
    StartCyc(ID taskName);
    virtual ~StartCyc();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual StartCyc *generateReverseCommand() override;
};

#endif