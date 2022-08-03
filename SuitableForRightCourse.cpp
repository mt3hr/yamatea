#include "PIDTracer.h"
#include "Walker.h"
#include "MotorCountPredicate.h"
#include "WheelController.h"

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

MotorCountPredicate *generateMotorCountPredicate(bool isRightCource, int count, WheelController *wheelController)
{
    if (isRightCource)
    {
        return new MotorCountPredicate(wheelController->getLeftWheel(), count);
    }
    else
    {
        return new MotorCountPredicate(wheelController->getRightWheel(), count);
    }
}
