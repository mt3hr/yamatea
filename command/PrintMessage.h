#ifndef PrintMessage_H
#define PrintMessage_H

#include "Command.h"
#include "string"
#include "vector"

using namespace std;

class PrintMessage : public Command
{
private:
    const string EOL_STR = string("\r\n");
    vector<string> messageLines;
    bool forcePrint;
    void print();

public:
    PrintMessage(vector<string> messageLines, bool forcePrint);
    void run() override;
    PrintMessage *generateReverseCommand() override;
};

#endif