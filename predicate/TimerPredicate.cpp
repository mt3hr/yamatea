#include "TimerPredicate.h"
#include "Predicate.h"
#include "RobotAPI.h"
#include "Clock.h"

TimerPredicate::TimerPredicate(uint64_t durationUsec){};

TimerPredicate::~TimerPredicate(){};

bool TimerPredicate::test(RobotAPI *robotAPI)
{
    return targetTime <= robotAPI->getClock()->now();
}

void TimerPredicate::preparation(RobotAPI *robotAPI)
{
    targetTime = robotAPI->getClock()->now() + durationUsec;
}

TimerPredicate *TimerPredicate::generateReversePredicate()
{
    return new TimerPredicate(durationUsec);
}