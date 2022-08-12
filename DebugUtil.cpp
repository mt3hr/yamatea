#include "DebugUtil.h"
#include "string"
#include "PrintMessage.h"
#include "vector"
#include "sstream"
#include "Setting.h"

using namespace std;

vector<string> messageLinesForDebugPrint;

void writeDebug(string str)
{
    if (!enablePrintDebugMessage)
    {
        return;
    }

    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += str;
}

void writeDebug(int i)
{
    if (!enablePrintDebugMessage)
    {
        return;
    }

    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << i;
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

void writeDebug(float f)
{
    if (!enablePrintDebugMessage)
    {
        return;
    }

    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << f;
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

// 改行する
void writeEndLineDebug()
{
    if (!enablePrintDebugMessage)
    {
        return;
    }
    messageLinesForDebugPrint.push_back("");
}

void flushDebug()
{
    if (!enablePrintDebugMessage)
    {
        return;
    }
    PrintMessage *printMessage = new PrintMessage(messageLinesForDebugPrint, true);
    printMessage->run();
    delete printMessage;
    vector<string> v;
    messageLinesForDebugPrint = v;
}