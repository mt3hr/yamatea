#ifndef ReleaseWheel_H
#define ReleaseWheel_H

#include "Command.h"
#include "RobotAPI.h"

class ReleaseWheel : public Command
{
private:
public:
    ReleaseWheel();
    virtual ~ReleaseWheel();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ReleaseWheel *generateReverseCommand() override;
};

#endif