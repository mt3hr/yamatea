#include "FinishedCommandPredicate.h"
#include "FinishConfirmable.h"

FinishedCommandPredicate::FinishedCommandPredicate(FinishConfirmable *fc)
{
    finishConfirmable = fc;
}

bool FinishedCommandPredicate::test()
{
    return finishConfirmable->isFinished();
}