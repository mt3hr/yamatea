#include "BrightnessReader.h"
#include "ColorSensor.h"
#include "PrintMessage.h"
#include "sstream"
#include "RobotAPI.h"

using namespace std;
using namespace ev3api;

BrightnessReader::BrightnessReader(){};

BrightnessReader::~BrightnessReader(){};

void BrightnessReader::run(RobotAPI *robotAPI)
{
    if (!lockedBrightnessValue)
    {
        brightness = robotAPI->getColorSensor()->getBrightness();
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            lockedBrightnessValue = true;
        }

        // 出力処理。変数名は雑。
        stringstream bs;

        bs << "brightness:" << float(brightness); // intのままだと出力されないのでfloatに変換する

        vector<string> messageLines;
        messageLines.push_back("Color ID Reader");
        messageLines.push_back("press right key lock value");
        messageLines.push_back(bs.str());

        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }
    else
    {
        if (!printedLockedBrightnessValue)
        {
            printedLockedBrightnessValue = true;
            // 出力処理。変数名は雑。
            stringstream bs;
            stringstream css;

            bs << "brightness:" << float(brightness); // intのままだと出力されないのでfloatに変換する

            vector<string> messageLines;
            messageLines.push_back("got Brightness");
            messageLines.push_back(bs.str());

            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
    }
}

void BrightnessReader::preparation(RobotAPI *robotAPI)
{
}

BrightnessReader *BrightnessReader::generateReverseCommand()
{
    return new BrightnessReader();
}
