#ifndef FinishedCommandPredicate_H
#define FinishedCommandPredicate_H

#include "Predicate.h"
#include "FinishConfirmable.h"

class FinishedCommandPredicate : public Predicate
{
private:
    FinishConfirmable *finishConfirmable;

public:
    FinishedCommandPredicate(FinishConfirmable *finishConfirmable);
    bool test() override;
};

#endif