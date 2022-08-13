#include "StartButtonPredicate.h"
#include "RobotAPI.h"

StartButtonPredicate::StartButtonPredicate()
{
}

bool StartButtonPredicate::test(RobotAPI *robotAPI)
{
    return robotAPI->getTouchSensor()->isPressed();
}