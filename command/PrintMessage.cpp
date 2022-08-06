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
        const char *message = messageLines[i].c_str();
        const char *messageEOLAppended = (messageLines[i] + EOL_STR).c_str();

        msg_f(message, i + 1);
        printf("%s", messageEOLAppended);
#ifdef PrintMessageForBlueTooth
        msg_bt(messageEOLAppended);
#endif
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