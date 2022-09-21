#include "DebugUtil.h"
#include "string"
#include "PrintMessage.h"
#include "vector"
#include "sstream"
#include "Setting.h"
#include "MusicalScore.h"
#include "ev3api.h"

using namespace std;
using namespace ev3api;

vector<string> messageLinesForDebugPrint;

void clearDebug()
{
    vector<string> v;
    messageLinesForDebugPrint = v;
}

void writeDebug(string str)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += str;
}

void writeDebug(int i)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << float(i);

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

void writeDebug(float f)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << float(f);

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

void writeDebug(uint16_t i)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << float(i);

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

void writeDebug(uint64_t i)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << float(i);

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

void writeDebug(int32_t i)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << float(i);

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

void writeDebug(bool b)
{
    if (messageLinesForDebugPrint.size() == 0)
    {
        messageLinesForDebugPrint.push_back("");
    }

    stringstream ss;
    ss.clear();
    ss.str("");
    ss << b;

    // 最後の行に追記する
    messageLinesForDebugPrint[messageLinesForDebugPrint.size() - 1] += ss.str();
}

// 改行する
void writeEndLineDebug()
{
    messageLinesForDebugPrint.push_back("");
}

void flushDebug(DEBUG_LEVEL level, RobotAPI *robotAPI)
{
    if (int(debugMessageLevel) >= int(level))
    {
        PrintMessage *printMessage = new PrintMessage(messageLinesForDebugPrint, true);
        printMessage->run(robotAPI);
        delete printMessage;
    }
    clearDebug();
}

void writeAndFlushDebug(string str, DEBUG_LEVEL level, RobotAPI *robotAPI)
{
    writeDebug(str);
    flushDebug(level, robotAPI);
}

void beepDebug()
{
    ev3_speaker_set_volume(beepNoteWhenCommandSwitching->getVolume());
    ev3_speaker_play_tone(beepNoteWhenCommandSwitching->getFrequency(), beepNoteWhenCommandSwitching->getDuration());
}

int ledDebugIndex = 0;
void ledDebug()
{
    if (ledDebugIndex > ((int)ledColorsWhenCommandSwitching.size()) - 1)
    {
        ledDebugIndex = 0;
    }
    ev3_led_set_color(ledColorsWhenCommandSwitching[ledDebugIndex]);
    ledDebugIndex++;
}