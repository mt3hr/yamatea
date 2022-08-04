#ifndef Stopper_H
#define Stopper_H

#include "Command.h"
#include "WheelController.h"

class Stopper : public Command
{
private:
    WheelController *wheelController;

public:
    Stopper(WheelController *wheelController);
    void run() override;
    Command *generateReverseCommand() override;
};

#endif
