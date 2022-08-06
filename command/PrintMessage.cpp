#include "PrintMessage.h"
#include "util.h"

PrintMessage::PrintMessage(string *ml)
{
    messageLines = ml;
}

void PrintMessage::run()
{
#ifdef PrintMessageMode
    int i = 0;
    for (; i < ((int)sizeof(messageLines)); i++)
    {
        const char *message = messageLines[i].c_str();
        const char *messageEOLAppended = (messageLines[i] + EOL_STR).c_str();

        msg_f(message, i + 1);
#ifdef PrintMessageForConsole
        printf("%s", messageEOLAppended);
#endif
#ifdef PrintMessageForBlueTooth
        msg_bt(messageEOLAppended);
#endif
    }
    for (; i < 7; i++)
    {
        msg_f("", i + 1);
    }
#endif
}

PrintMessage *PrintMessage::generateReverseCommand()
{
    return new PrintMessage(messageLines);
}

void PrintMessage::setMessageLines(string *ml)
{
    messageLines = ml;
}