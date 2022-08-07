#ifndef Stopper_H
#define Stopper_H

#include "Command.h"
#include "WheelController.h"

// Stopper
// ロボットの左右車輪を停止するコマンド
// 
// 実方
class Stopper : public Command
{
private:
    WheelController *wheelController;

public:
    Stopper(WheelController *wheelController);
    void run() override;
    Stopper *generateReverseCommand() override;
};

#endif
