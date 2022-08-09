#ifndef CommandAndPreicate_H
#define CommandAndPreicate_H

#include "Command.h"
#include "Predicate.h"
#include "Handler.h"

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
    Handler *preHandler;

public:
    CommandAndPredicate(Command *command, Predicate *predicate, Handler *preHandler);
    Command *getCommand();
    Predicate *getPredicate();
    Handler *getPreHandler();
};

#endif