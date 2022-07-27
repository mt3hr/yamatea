#include "StartButtonPredicate.h"

StartButtonPredicate::StartButtonPredicate(TouchSensor *ts)
{
    touchSensor = ts;
}

bool StartButtonPredicate::test()
{
    return touchSensor->isPressed();
}