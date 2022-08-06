#include "PrintMessage.h"
#include "util.h"

PrintMessage::PrintMessage(string *ml, bool fp)
{
    messageLines = ml;
    forcePrint = fp;
}

void PrintMessage::run()
{
    int i = 0;

    // 強制出力ここから
    if (forcePrint)
    {
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
    }
    // 強制出力ここまで

// 強制でない出力ここから
#ifdef PrintMessageMode
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
    // 強制でない出力ここまで
}

PrintMessage *PrintMessage::generateReverseCommand()
{
    return new PrintMessage(messageLines, forcePrint);
}

void PrintMessage::setMessageLines(string *ml)
{
    messageLines = ml;
}