#include "StartButtonPredicate.h"
#include "RobotAPI.h"

StartButtonPredicate::StartButtonPredicate()
{
}

bool StartButtonPredicate::test(RobotAPI *robotAPI)
{
    return robotAPI->getTouchSensor()->isPressed();
}

void StartButtonPredicate::preparation(RobotAPI *robotAPI)
{
}

StartButtonPredicate *StartButtonPredicate::generateReversePredicate()
{
    return new StartButtonPredicate();
}