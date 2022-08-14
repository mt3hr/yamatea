#include "CommandAndPredicate.h"
#include "Command.h"
#include "Predicate.h"

CommandAndPredicate::CommandAndPredicate(Command *c, Predicate *p)
{
    command = c;
    predicate = p;
}

Command *CommandAndPredicate::getCommand()
{
    return command;
}

Predicate *CommandAndPredicate::getPredicate()
{
    return predicate;
}
