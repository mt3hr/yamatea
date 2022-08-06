#include "PrintMessage.h"
#include "util.h"

PrintMessage::PrintMessage(string *ml)
{
    messageLines = ml;
}

void PrintMessage::run()
{
    int i = 0;
    for (; i < ((int)sizeof(messageLines)); i++)
    {
        msg_f(messageLines[i].c_str(), i + 1);
    }
    for (; i < 7; i++)
    {
        msg_f("", i + 1);
    }
}

PrintMessage *PrintMessage::generateReverseCommand()
{
    return new PrintMessage(messageLines);
}

void PrintMessage::setMessageLines(string *ml)
{
    messageLines = ml;
}