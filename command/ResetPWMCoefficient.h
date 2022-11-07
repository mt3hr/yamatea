#ifndef ResetPWMCoefficient_H
#define ResetPWMCoefficient_H

#include "SetPWMCoefficient.h"
#include "Command.h"
#include "RobotAPI.h"

class ResetPWMCoefficient : public SetPWMCoefficient
{
private:
public:
    ResetPWMCoefficient();
    virtual ~ResetPWMCoefficient();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ResetPWMCoefficient *generateReverseCommand() override;
};

#endif