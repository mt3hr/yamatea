#ifndef FinishedCommandPredicate_H
#define FinishedCommandPredicate_H

#include "Predicate.h"
#include "FinishConfirmable.h"
#include "RobotAPI.h"

class FinishedCommandPredicate : public Predicate
{
private:
    FinishConfirmable *finishConfirmable;

public:
    FinishedCommandPredicate(FinishConfirmable *finishConfirmable);
    bool test(RobotAPI *robotAPI) override;
    void preparation(RobotAPI *robotAPI) override;
};

#endif