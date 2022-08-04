#ifndef PrintStartedMessage_H
#define PrintStartedMessage_H

#include "Command.h"

class PrintStartedMessage : public Command
{
private:
public:
    PrintStartedMessage();
    void run() override;
    PrintStartedMessage *generateReverseCommand() override;
};

#endif