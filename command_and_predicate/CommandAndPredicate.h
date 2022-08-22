#ifndef CommandAndPreicate_H
#define CommandAndPreicate_H

#include "Command.h"
#include "Predicate.h"

// CommandAndPredicate
// CommandとPredicateの1セット
// 指定角度旋回コマンドや孤をなぞるコマンドを生成するときにこいつがあると便利
//
// 実方
class CommandAndPredicate
{
private:
    Command *command;
    Predicate *predicate;

public:
    CommandAndPredicate();
    virtual ~CommandAndPredicate();
    virtual Command *getCommand();
    virtual Predicate *getPredicate();
    virtual void setCommand(Command *command);
    virtual void setPredicate(Predicate *predicate);
};

#endif