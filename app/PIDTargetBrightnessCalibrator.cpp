#include "PIDTargetBrightnessCalibrator.h"
#include "Handler.h"
#include "util.h"
#include "Clock.h"
#include "SonarSensor.h" //TODOけして

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

// rgb_raw_t rgb;       // TODO けして
// bool gotRGB = false; // TODO けして
int16_t distanceValue = 0; // TODO けして
bool gotDistance; // TODO けして
SonarSensor sonorSensor(PORT_3); // TODO けして

void PIDTargetBrightnessCalibrator::run()
{
    // TODO けしてここから
    if (!gotDistance)
    {
        msg_f("distance", 1);
        msg_f("press right key", 2);
        msg_f("     read distance", 3);
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            distanceValue = sonorSensor.getDistance();
            gotDistance = true;
        }
    }
    else
    {
        char dStr[20];
        sprintf(dStr, "distance:%d", distanceValue);
        msg_f(dStr, 1);
    }

    // TODO けしてここまで

    /*
    // TODO けしてここから
    if (!gotRGB)
    {
        msg_f("rawRGB", 1);
        msg_f("press right key", 2);
        msg_f("     read rgb", 3);
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            colorSensor->getRawColor(rgb);
            gotRGB = true;
        }
    }
    else
    {
        char rStr[20];
        char gStr[20];
        char bStr[20];
        sprintf(rStr, "r:%d", rgb.r);
        sprintf(gStr, "g:%d", rgb.g);
        sprintf(bStr, "b:%d", rgb.b);
        msg_f(rStr, 1);
        msg_f(gStr, 2);
        msg_f(bStr, 3);
    }
    // TODO けしてここまで
    */

    /*TODO コメントアウトここから
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
        //TODO コメントアウトここまで*/
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