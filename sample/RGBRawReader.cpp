#include "RGBRawReader.h"
#include "ColorSensor.h"
#include "PrintMessage.h"
#include "sstream"

using namespace ev3api;
using namespace std;

RGBRawReader::RGBRawReader(ColorSensor *cs)
{
    colorSensor = cs;
}

void RGBRawReader::run()
{
    if (!lockedRGBRawValue)
    {
        colorSensor->getRawColor(rgbRaw);
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            lockedRGBRawValue = true;
        }

        // 出力処理。変数名は雑。
        stringstream rs;
        stringstream gs;
        stringstream bs;

        rs << "r:" << float(rgbRaw.r); // intのままだと出力されないのでfloatに変換する
        gs << "g:" << float(rgbRaw.g); // intのままだと出力されないのでfloatに変換する
        bs << "b:" << float(rgbRaw.b); // intのままだと出力されないのでfloatに変換する

        vector<string> messageLines;
        messageLines.push_back("Raw RBG Reader");
        messageLines.push_back("press right key lock value");
        messageLines.push_back(rs.str());
        messageLines.push_back(gs.str());
        messageLines.push_back(bs.str());

        PrintMessage printMessage(messageLines, true);
        printMessage.run();
    }
    else
    {
        if (!printedLockedRGBRawValue)
        {
            printedLockedRGBRawValue = true;
            // 出力処理。変数名は雑。
            stringstream rs;
            stringstream gs;
            stringstream bs;

            rs << "r:" << float(rgbRaw.r); // intのままだと出力されないのでfloatに変換する
            gs << "g:" << float(rgbRaw.g); // intのままだと出力されないのでfloatに変換する
            bs << "b:" << float(rgbRaw.b); // intのままだと出力されないのでfloatに変換する

            vector<string> messageLines;
            messageLines.push_back("got RGB raw");
            messageLines.push_back(rs.str());
            messageLines.push_back(gs.str());
            messageLines.push_back(bs.str());

            PrintMessage printMessage(messageLines, true);
            printMessage.run();
        }
    }
}

RGBRawReader *RGBRawReader::generateReverseCommand()
{
    return new RGBRawReader(colorSensor);
}