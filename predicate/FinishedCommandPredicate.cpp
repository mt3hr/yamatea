#include "FinishedCommandPredicate.h"
#include "FinishConfirmable.h"

FinishedCommandPredicate::FinishedCommandPredicate(FinishConfirmable *fc)
{
    finishConfirmable = fc;
}

bool FinishedCommandPredicate::test(RobotAPI *robotAPI)
{
    return finishConfirmable->isFinished();
}

void FinishedCommandPredicate::preparation(RobotAPI *robotAPI)
{
    return;
}

FinishedCommandPredicate *FinishedCommandPredicate::generateReversePredicate()
{
    return new FinishedCommandPredicate(finishConfirmable);
}