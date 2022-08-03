#ifndef CommandAndPreicate_H
#define CommandAndPreicate

#include "Command.h"
#include "Predicate.h"

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