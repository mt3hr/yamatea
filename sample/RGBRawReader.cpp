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
        colorSensor->getRawColor(rgbRaw);
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            gotRGB = true;
        }
        char rStr[20];
        char gStr[20];
        char bStr[20];
        sprintf(rStr, "r:%d", rgbRaw.r);
        sprintf(gStr, "g:%d", rgbRaw.g);
        sprintf(bStr, "b:%d", rgbRaw.b);

        msg_f("Raw RGB Reader", 1);
        msg_f("press right key lock value", 2);
        msg_f(rStr, 3);
        msg_f(gStr, 4);
        msg_f(bStr, 5);
        msg_f("", 6);
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