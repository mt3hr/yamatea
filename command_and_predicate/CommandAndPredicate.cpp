#include "CommandAndPredicate.h"
#include "Command.h"
#include "Predicate.h"

CommandAndPredicate::CommandAndPredicate()
{
}

Command *CommandAndPredicate::getCommand()
{
    return command;
}

Predicate *CommandAndPredicate::getPredicate()
{
    return predicate;
}

void CommandAndPredicate::setCommand(Command *command)
{
    this->command = command;
}

void CommandAndPredicate::setPredicate(Predicate *predicate)
{
    this->predicate = predicate;
}
