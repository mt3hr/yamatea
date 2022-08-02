#include "PIDTargetBrightnessCalibrator.h"
#include "Handler.h"
#include "util.h"
#include "Clock.h"
// #include "SonarSensor.h" //TODOけして

using namespace ev3api;
using namespace std;

int sleepDuration = 1000 * 500;

PIDTargetBrightnessCalibrator::PIDTargetBrightnessCalibrator(ColorSensor *cs, Clock *c)
{
    colorSensor = cs;
    clock = c;
};

void PIDTargetBrightnessCalibrator::readWhiteFromColorSensor()
{
    readedWhite = true;
    white = colorSensor->getBrightness();
}

void PIDTargetBrightnessCalibrator::readBlackFromColorSensor()
{
    readedBlack = true;
    black = colorSensor->getBrightness();
}

void PIDTargetBrightnessCalibrator::run()
{
    if (!isReadedBlack())
    {
        msg_f("calibrating", 1);
        msg_f("press right key", 2);
        msg_f("     read black", 3);
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readBlackFromColorSensor();
            clock->sleep(sleepDuration);
        }
    }
    else if (!isReadedWhite())
    {
        msg_f("calibrating", 1);
        msg_f("press right key", 2);
        msg_f("     read white", 3);
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readWhiteFromColorSensor();
            clock->sleep(sleepDuration);
        }
    }
    else
    {
        if (!handlerExecuted)
        {
            handlerExecuted = true;
            for (int i = 0; i < ((int)handlers.size()); i++)
            {
                Handler *handler = handlers[i];
                handler->handle();
            }
        }

        char bStr[20];
        char wStr[20];
        msg_f("calibrated!", 1);
        sprintf(bStr, "black:%d", getBlack());
        sprintf(wStr, "white:%d", getWhite());
        msg_f(bStr, 2);
        msg_f(wStr, 3);
        msg_f("", 4);
        msg_f("press touch sensor", 5);
    }
}

PIDTargetBrightnessCalibrator *PIDTargetBrightnessCalibrator::generateReverseCommand()
{
    return new PIDTargetBrightnessCalibrator(colorSensor, clock);
}

int PIDTargetBrightnessCalibrator::getBlack()
{
    return black;
}

int PIDTargetBrightnessCalibrator::getWhite()
{
    return white;
}

bool PIDTargetBrightnessCalibrator::isReadedBlack()
{
    return readedBlack;
}

bool PIDTargetBrightnessCalibrator::isReadedWhite()
{
    return readedWhite;
}

void PIDTargetBrightnessCalibrator::addRoadedHandler(Handler *h)
{
    handlers.push_back(h);
}