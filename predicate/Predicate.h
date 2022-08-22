#ifndef Predicate_H
#define Predicate_H

#include "RobotAPI.h"

// Predicate
// CommandExecutorでコマンドを渡す時に同時に渡すもの。
// コマンドの終了条件を判定するために使われる。
//
// 実方
class Predicate
{
public:
    Predicate();
    virtual ~Predicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual Predicate *generateReversePredicate();
};
#endif