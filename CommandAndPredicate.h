#ifndef CommandAndPreicate_H
#define CommandAndPreicate_H

#include "Command.h"
#include "Predicate.h"

// CommandAndPredicate
// CommandとPredicateの1セット
// 指定角度旋回コマンドを生成するときにこいつがあると便利
// 
// 実方
class CommandAndPredicate
{
private:
    Command *command;
    Predicate *predicate;

public:
    CommandAndPredicate(Command *command, Predicate *predicate);
    Command *getCommand();
    Predicate *getPredicate();
};

#endif