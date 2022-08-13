#include "PIDTracer.h"
#include "Walker.h"
#include "MotorCountPredicate.h"

PIDTracer *ifRightThenReverseCommand(PIDTracer *pidTracer, bool isRightCource)
{
    if (isRightCource)
    {
        PIDTracer *reversed = pidTracer->generateReverseCommand();
        return reversed;
    }
    else
    {
        return pidTracer;
    }
}

Walker *ifRightThenReverseCommand(Walker *walker, bool isRightCource)
{
    if (isRightCource)
    {
        Walker *reversed = walker->generateReverseCommand();
        return reversed;
    }
    else
    {
        return walker;
    }
}

MotorCountPredicate *generateMotorCountPredicate(bool isRightCource, int count, RobotAPI *robotAPI)
{
    if (isRightCource)
    {
        return new MotorCountPredicate(robotAPI->getLeftWheel(), count, false);
    }
    else
    {
        return new MotorCountPredicate(robotAPI->getRightWheel(), count, false);
    }
}
