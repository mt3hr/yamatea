#ifndef ResetMeasAngle_H
#define ResetMeasAngle_H

#include "Command.h"
#include "RobotAPI.h"

class ResetMeasAngle : public Command
{
private:
public:
    ResetMeasAngle();
    virtual ~ResetMeasAngle();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ResetMeasAngle *generateReverseCommand() override;
};

#endif