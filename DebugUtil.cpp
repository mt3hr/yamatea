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
    // 最後の行に追記する
    messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] = messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] + str;
}

void writeDebug(int i)
{
    if (!enablePrintDebugMessage)
    {
        return;
    }
    stringstream ss;
    ss.clear();
    ss.str("");
    ss << i;
    messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] = messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] + ss.str();
}

void writeDebug(float f)
{
    if (!enablePrintDebugMessage)
    {
        return;
    }
    stringstream ss;
    ss.clear();
    ss.str("");
    ss << f;
    messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] = messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] + ss.str();
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