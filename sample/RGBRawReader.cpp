#include "RGBRawReader.h"
#include "ColorSensor.h"
#include "util.h"

using namespace ev3api;

RGBRawReader::RGBRawReader(ColorSensor *cs)
{
    colorSensor = cs;
}

void RGBRawReader::run()
{
    if (!gotRGB)
    {
        msg_f("Raw RGB Reader", 1);
        msg_f("press right key", 2);
        msg_f("     read rgb", 3);
        msg_f("", 4);
        msg_f("", 5);
        msg_f("", 6);
        msg_f("", 7);

        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            colorSensor->getRawColor(rgbRaw);
            gotRGB = true;
        }
    }
    else
    {
        char rStr[20];
        char gStr[20];
        char bStr[20];
        sprintf(rStr, "r:%d", rgbRaw.r);
        sprintf(gStr, "g:%d", rgbRaw.g);
        sprintf(bStr, "b:%d", rgbRaw.b);
        msg_f("got rgb raw", 1);
        msg_f(rStr, 2);
        msg_f(gStr, 3);
        msg_f(bStr, 4);
    }
}

Command *RGBRawReader::generateReverseCommand()
{
    return new RGBRawReader(colorSensor);
}