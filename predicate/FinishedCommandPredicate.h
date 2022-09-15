#ifndef FinishedCommandPredicate_H
#define FinishedCommandPredicate_H

#include "Predicate.h"
#include "FinishConfirmable.h"
#include "RobotAPI.h"

// FinishedCommandPredicate
// 「終了状態」を持つコマンドからPredicateを生成するためのクラス。
// 終了状態を持つコマンドは、FinishConfirmableインターフェースを実装している。
//
// 実方
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
    virtual void setTarget(FinishConfirmable *target);
};

#endif