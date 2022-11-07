#ifndef SetPWMCoefficient_H
#define SetPWMCoefficient_H

#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;

class SetPWMCoefficient : public Command
{
private:
    float calcPWMCoefficient();

public:
    SetPWMCoefficient();
    virtual ~SetPWMCoefficient();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual SetPWMCoefficient *generateReverseCommand() override;
};
#endif