#include "PrintMessage.h"
#include "vector"
#include "util.h"

using namespace std;

PrintMessage::PrintMessage(vector<const char *> ml)
{
    messageLines = ml;
}

void PrintMessage::run()
{
    int i = 0;
    for (; i < ((int)sizeof(messageLines)); i++)
    {
        msg_f(messageLines[i], i + 1);
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
