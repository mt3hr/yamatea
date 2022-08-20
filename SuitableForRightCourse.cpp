#include "PIDTracer.h"
#include "Walker.h"
#include "LeftWheelCountPredicate.h"
#include "RightWheelCountPredicate.h"
#include "Setting.h"

PIDTracer *ifRightThenReverseCommand(PIDTracer *pidTracer)
{
#ifdef LeftCourceMode
    return pidTracer;
#endif
#ifdef RightCourceMode
    PIDTracer *reversed = pidTracer->generateReverseCommand();
    delete pidTracer;
    return reversed;
#endif
}

Walker *ifRightThenReverseCommand(Walker *walker)
{
#ifdef LeftCourceMode
    return walker;
#endif
#ifdef RightCourceMode
    Walker *reversed = walker->generateReverseCommand();
    delete walker;
    return reversed;
#endif
}

Predicate *generateWheelCountPredicate(int count)
{

#ifdef LeftCourceMode
    return new LeftWheelCountPredicate(count);
#endif
#ifdef RightCourceMode
    return new RightWheelCountPredicate(count);
#endif
}
