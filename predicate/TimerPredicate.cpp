#include "TimerPredicate.h"
#include "Predicate.h"
#include "RobotAPI.h"
#include "Clock.h"
#include "DebugUtil.h"

TimerPredicate::TimerPredicate(uint64_t durationUsec)
{
    this->durationUsec = durationUsec;
};

TimerPredicate::~TimerPredicate(){};

bool TimerPredicate::test(RobotAPI *robotAPI)
{
    uint64_t now = robotAPI->getClock()->now();
    bool result = targetTime <= now;
    writeDebug("targetTime: ");
    writeDebug(targetTime);
    writeEndLineDebug();
    writeDebug("now: ");
    writeDebug(now);
    flushDebug(TRACE, robotAPI);
    return result;
}

void TimerPredicate::preparation(RobotAPI *robotAPI)
{
    targetTime = robotAPI->getClock()->now() + durationUsec;
}

TimerPredicate *TimerPredicate::generateReversePredicate()
{
    return new TimerPredicate(durationUsec);
}