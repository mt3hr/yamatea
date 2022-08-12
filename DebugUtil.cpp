#include "DebugUtil.h"
#include "string"
#include "PrintMessage.h"
#include "vector"
#include "sstream"

using namespace std;

vector<string> messageLinesForDebugPrint;

void writeDebug(string str)
{
    // 最後の行に追記する
    messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] = messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] + str;
}

void writeDebug(int i)
{
    stringstream ss;
    ss.clear();
    ss.str("");
    ss << i;
    messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] = messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] + ss.str();
}

void writeDebug(float f)
{
    stringstream ss;
    ss.clear();
    ss.str("");
    ss << f;
    messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] = messageLinesForDebugPrint[sizeof(messageLinesForDebugPrint) - 1] + ss.str();
}

// 改行する
void writeEndLineDebug()
{
    messageLinesForDebugPrint.push_back("");
}

void flushDebug()
{
    PrintMessage *printMessage = new PrintMessage(messageLinesForDebugPrint, true);
    printMessage->run();
    delete printMessage;
    vector<string> v;
    messageLinesForDebugPrint = v;
}