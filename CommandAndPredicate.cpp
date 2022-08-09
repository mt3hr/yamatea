#include "CommandAndPredicate.h"
#include "Command.h"
#include "Predicate.h"
#include "Handler.h"

CommandAndPredicate::CommandAndPredicate(Command *c, Predicate *p, Handler *h)
{
    command = c;
    predicate = p;
    preHandler = h;
}

Command *CommandAndPredicate::getCommand()
{
    return command;
}

Predicate *CommandAndPredicate::getPredicate()
{
    return predicate;
}

Handler *CommandAndPredicate::getPreHandler() {
    return preHandler;
}