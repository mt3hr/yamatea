#include "PrintStartedMessage.h"
#include "util.h"

PrintStartedMessage::PrintStartedMessage()
{
}

void PrintStartedMessage::run()
{
    msg_f("Running!!\r\n", 1);
    msg_f("GOGOGO!!\r\n", 2);
    msg_f("", 3);
    msg_f("", 4);
    msg_f("", 5);
    msg_f("", 6);
    msg_f("", 7);
}

PrintStartedMessage *PrintStartedMessage::generateReverseCommand()
{
    return new PrintStartedMessage();
}