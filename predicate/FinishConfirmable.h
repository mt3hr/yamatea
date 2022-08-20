#ifndef FinishConfirmable_H
#define FinishConfirmable_H

class FinishConfirmable
{
public:
    FinishConfirmable();
    virtual ~FinishConfirmable();
    virtual bool isFinished();
};

#endif