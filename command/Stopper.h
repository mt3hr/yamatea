#ifndef Stopper_H
#define Stopper_H

#include "Command.h"
#include "RobotAPI.h"

// Stopper
// ロボットの左右車輪を停止するコマンド
// 
// 実方
class Stopper : public Command
{
private:

public:
    Stopper();
    void run(RobotAPI *robotAPI) override;
    Stopper *generateReverseCommand() override;
};

#endif
