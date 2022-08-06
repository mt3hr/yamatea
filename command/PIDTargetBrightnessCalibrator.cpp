#include "PIDTargetBrightnessCalibrator.h"
#include "Handler.h"
#include "Clock.h"
#include "string"

using namespace ev3api;
using namespace std;

int sleepDuration = 1000 * 500;

PIDTargetBrightnessCalibrator::PIDTargetBrightnessCalibrator(ColorSensor *cs, Clock *c)
{
    colorSensor = cs;
    clock = c;
    string messageLines[] = {"target brightness calibrating"};
    printMessage = new PrintMessage(messageLines, true);
};

PIDTargetBrightnessCalibrator::~PIDTargetBrightnessCalibrator()
{
    delete printMessage;
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
            string messageLines[] = {
                "calibrating",
                "press right key",
                "     read black",
            };
            printMessage->setMessageLines(messageLines);
            printMessage->run();
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
            string messageLines[] = {
                "calibrating",
                "press right key",
                "     read white",
            };
            printMessage->setMessageLines(messageLines);
            printMessage->run();
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
            char bStr[20];
            char wStr[20];
            sprintf(bStr, "black:%d\r\n", getBlack());
            sprintf(wStr, "white:%d\r\n", getWhite());

            string messageLines[] = {
                "calibrated!",
                string(bStr),
                string(wStr),
                "",
                "press touch sensor",
            };
            printMessage->setMessageLines(messageLines);
            printMessage->run();
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