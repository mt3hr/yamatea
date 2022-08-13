#ifndef Predicate_H
#define Predicate_H

#include "RobotAPI.h"

// Predicate
// CommandExecutorでコマンドを渡す時に同時に渡すもの。
// コマンドの終了条件を判定するために使われる。
// 既知のPredicate: MotorCountPredicate
//                  StartButtonPredicate
//
// 実方
class Predicate
{
public:
    virtual ~Predicate();
    virtual bool test(RobotAPI *robotAPI);
};
#endif