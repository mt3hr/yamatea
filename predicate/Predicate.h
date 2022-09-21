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

    // 条件を満たしていたらtrueを返す。オーバーライドして使って。
    virtual bool test(RobotAPI *robotAPI);

    // 事前準備処理。Predicateが呼び出される直前に一度だけ呼び出される。オーバーライドして使って
    virtual void preparation(RobotAPI *robotAPI);

    // Predicateを左右反転する。オーバーライドして使って
    virtual Predicate *generateReversePredicate();
};
#endif