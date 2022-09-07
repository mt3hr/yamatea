#ifndef ResetGyroSensor_H
#define ResetGyroSensor_H

#include "Command.h"
#include "RobotAPI.h"

class ResetGyroSensor : public Command
{
private:
public:
    ResetGyroSensor();
    virtual ~ResetGyroSensor();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ResetGyroSensor *generateReverseCommand() override;
};

#endif