#ifndef PrintMessage_H
#define PrintMessage_H

#include "Command.h"
#include "string"

using namespace std;

class PrintMessage : public Command
{
private:
    string *messageLines;

public:
    PrintMessage(string *messageLines);
    void run() override;
    PrintMessage *generateReverseCommand() override;
};

#endif