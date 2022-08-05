#ifndef PrintMessage_H
#define PrintMessage_H

#include "vector"
#include "Command.h"

using namespace std;

class PrintMessage : public Command
{
private:
    vector<const char *> messageLines;

public:
    PrintMessage(vector<const char *> messageLines);
    void run() override;
    PrintMessage *generateReverseCommand() override;
};

#endif