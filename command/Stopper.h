#ifndef Stopper_H
#define Stopper_H

#include "Command.h"
#include "RobotAPI.h"

// Stopper
// ロボットのモータをすべて停止させるコマンド
// 
// 実方
class Stopper : public Command
{
private:

public:
    Stopper();
    virtual ~Stopper();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Stopper *generateReverseCommand() override;
};

#endif
