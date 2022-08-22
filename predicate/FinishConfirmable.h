#ifndef FinishConfirmable_H
#define FinishConfirmable_H

// FinishConfirmable
// 「終了状態」を持つコマンドに実装されるインターフェース。
// 実装すると、FinishedCommandPredicateコンストラクタに渡すことによってPredicateを生成できる。
// 
// 実方
class FinishConfirmable
{
public:
    FinishConfirmable();
    virtual ~FinishConfirmable();
    virtual bool isFinished();
};

#endif