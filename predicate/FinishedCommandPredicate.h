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
    virtual ~FinishedCommandPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual FinishedCommandPredicate *generateReversePredicate() override;
};

#endif