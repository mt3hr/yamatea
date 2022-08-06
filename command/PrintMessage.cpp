#include "PrintMessage.h"
#include "util.h"
#include "vector"
#include "string"
#include "Setting.h"

using namespace std;

PrintMessage::PrintMessage(vector<string> ml, bool fp)
{
    // 一応ディープコピーしておきます。
    vector<string> messageLinesTemp;
    for (int i = 0; i < ((int)ml.size()); i++)
    {
        string message = string(ml[i].c_str());
        messageLinesTemp.push_back(message);
    }
    messageLines = messageLinesTemp;

    forcePrint = fp;
}

void PrintMessage::print()
{
    int i = 0;
    for (; i < ((int)messageLines.size()); i++)
    {
        string messageLine = messageLines[i];
        string messageLineEOLAppended = "";

        if (enablePrintMessageForConsole || enablePrintMessageForBluetooth)
        {
            messageLineEOLAppended.append(messageLines[i]);
            messageLineEOLAppended.append(EOL_STR);
        }

        // 出力処理
        msg_f(messageLine, i + 1);

        if (enablePrintMessageForConsole)
        {
            printf("%s", messageLineEOLAppended.c_str());
        }

#if defined(EnableBluetooth)
        if (enablePrintMessageForBluetooth)
        {
            msg_bt(messageLineEOLAppended);
        }
#endif
    }

    // 下の行の上書き処理。
    // 前のメッセージ行数が多いときにこの処理を挟まないと、前のメッセージが残り続ける。
    for (; i < 6; i++)
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
        if (enablePrintMessageMode)
        {
            print();
        }
    }
}

PrintMessage *PrintMessage::generateReverseCommand()
{
    return new PrintMessage(messageLines, forcePrint);
}
