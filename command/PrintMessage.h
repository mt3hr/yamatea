#ifndef PrintMessage_H
#define PrintMessage_H

#include "Command.h"
#include "string"

using namespace std;

class PrintMessage : public Command
{
    const string EOL_STR = "\r\n";

private:
    string *messageLines;
    bool forcePrint;
    void print();

public:
    PrintMessage(string *messageLines, bool forcePrint);
    void run() override;
    PrintMessage *generateReverseCommand() override;
    void setMessageLines(string *messageLines);
};

#endif