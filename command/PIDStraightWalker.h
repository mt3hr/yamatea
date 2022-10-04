#ifndef PIDStraightWalker_H
#define PIDStraightWalker_H

#include "Command.h"
#include "RobotAPI.h"

class PIDStraightWalker : public Command
{
private:
    float pwm;
    float kp;
    float ki;
    float kd;
    float dt;
    int wheelDifference = 0;
    int targetDifference = 0;
    int trueTargetDifference = 0;
    bool inited = false;

    float p;
    float i;
    float d;
    float pid;
    float beforeP = 0;
    float rightPower;
    float leftPower;
    bool back = false;

public:
    PIDStraightWalker(int pwm, float kp, float ki, float kd, float dt);
    virtual ~PIDStraightWalker();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual PIDStraightWalker *generateReverseCommand();
    virtual void setTargetDifferenceWheelCount(int targetDifference);
};
#endif