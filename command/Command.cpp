#include "Command.h"
#include "RobotAPI.h"

Command::~Command()
{
}

void Command::run(RobotAPI *robotAPI)
{
    return;
}

void Command::preparation(RobotAPI *robotAPI)
{
    return;
}

Command *Command::generateReverseCommand()
{
    return new Command();
}