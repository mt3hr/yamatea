#include "ColorIDReader.h"
#include "ColorSensor.h"
#include "PrintMessage.h"
#include "sstream"
#include "RobotAPI.h"

using namespace std;
using namespace ev3api;

string colorIDToString(colorid_t colorID)
{
    switch ((int)colorID)
    {
    case COLOR_NONE:
        return "NONE";
    case COLOR_BLACK:
        return "BLACK";
    case COLOR_BLUE:
        return "BLUE";
    case COLOR_GREEN:
        return "GREEN";
    case COLOR_YELLOW:
        return "YELLOW";
    case COLOR_RED:
        return "RED";
    case COLOR_WHITE:
        return "WHITE";
    case COLOR_BROWN:
        return "BROWM";
    }
    return "NONE";
}

ColorIDReader::ColorIDReader()
{
};

ColorIDReader::~ColorIDReader()
{
};

void ColorIDReader::run(RobotAPI *robotAPI)
{
    if (!lockedColorIDValue)
    {
        colorID = robotAPI->getColorSensor()->getColorNumber();
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            lockedColorIDValue = true;
        }

        // 出力処理。変数名は雑。
        stringstream cs;
        stringstream css;

        cs << "color id:" << float(colorID); // intのままだと出力されないのでfloatに変換する
        css << "color name: " << colorIDToString(colorID);

        vector<string> messageLines;
        messageLines.push_back("Color ID Reader");
        messageLines.push_back("press right key lock value");
        messageLines.push_back(cs.str());
        messageLines.push_back(css.str());

        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }
    else
    {
        if (!printedLockedColorIDValue)
        {
            printedLockedColorIDValue = true;
            // 出力処理。変数名は雑。
            stringstream cs;
            stringstream css;

            cs << "color id:" << float(colorID); // intのままだと出力されないのでfloatに変換する
            css << "color name: " << colorIDToString(colorID);

            vector<string> messageLines;
            messageLines.push_back("got Color ID");
            messageLines.push_back(cs.str());
            messageLines.push_back(css.str());

            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
    }
}

void ColorIDReader::preparation(RobotAPI *robotAPI)
{
}

ColorIDReader *ColorIDReader::generateReverseCommand()
{
    return new ColorIDReader();
}
