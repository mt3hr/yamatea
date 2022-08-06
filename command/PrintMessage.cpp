#include "PrintMessage.h"
#include "util.h"
#include "Setting.h"
#include "string"

using namespace std;

PrintMessage::PrintMessage(string *ml, bool fp)
{
    messageLines = ml;
    forcePrint = fp;
}

void PrintMessage::print()
{
    int i = 0;
    for (; i < ((int)sizeof(messageLines)); i++)
    {
        const char *message = messageLines[i].c_str();
        const char *messageEOLAppended;

        if (enablePrintMessageForConsole || enablePrintMessageForConsole)
        {
            string messageEOLAppendedStr = "";
            /* //TODO なんかここ消すとうまく動くんだよな。なんで？
            messageEOLAppendedStr.append(messageLines[i]);
            messageEOLAppendedStr.append(EOL_STR);
            messageEOLAppended = messageEOLAppendedStr.c_str();
            */
        }

        msg_f(message, i + 1);

        if (enablePrintMessageForConsole)
        {
            printf("%s", messageEOLAppended);
        }
        if (enablePrintMessageForBluetooth)
        {
            msg_bt(messageEOLAppended);
        }
    }
    for (; i < 7; i++)
    {
        msg_f("", i + 1);
    }
}

void PrintMessage::run()
{
    if (forcePrint)
    {
        print();
    }
    else
    {
#ifdef PrintMessageMode
        print();
#endif
    }
}

PrintMessage *PrintMessage::generateReverseCommand()
{
    return new PrintMessage(messageLines, forcePrint);
}

void PrintMessage::setMessageLines(string *ml)
{
    messageLines = ml;
}