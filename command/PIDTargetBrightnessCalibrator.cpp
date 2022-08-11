#include "PIDTargetBrightnessCalibrator.h"
#include "PrintMessage.h"
#include "Handler.h"
#include "Clock.h"
#include "string"
#include "sstream"
#include "vector"

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
        if (!printedReadBlackMessage)
        {
            printedReadBlackMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read black brightness");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run();
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readBlackFromColorSensor();
            clock->sleep(sleepDuration);
        }
    }
    else if (!isReadedWhite())
    {
        if (!printedReadWhiteMessage)
        {
            printedReadWhiteMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read white brightness");
            messageLines.push_back(" from color sensor");

            PrintMessage printMessage(messageLines, true);
            printMessage.run();
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readWhiteFromColorSensor();
            clock->sleep(sleepDuration);
        }
    }
    else
    {
        if (!executedHandler)
        {
            executedHandler = true;
            for (int i = 0; i < ((int)handlers.size()); i++)
            {
                Handler *handler = handlers[i];
                handler->handle();
            }
        }

        if (!printedCalibratedMessage)
        {
            printedCalibratedMessage = true;
            stringstream bs;
            stringstream ws;

            bs << "black:" << getBlack();
            ws << "white:" << getWhite();

            vector<string> messageLines;
            messageLines.push_back("calibrated!");
            messageLines.push_back(bs.str());
            messageLines.push_back(ws.str());
            messageLines.push_back("");
            messageLines.push_back("press touch sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run();
        }
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