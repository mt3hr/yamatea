#include "PIDTargetBrightnessCalibrator.h"
#include "PrintMessage.h"
#include "Clock.h"
#include "string"
#include "sstream"
#include "vector"
#include "RobotAPI.h"
#include "PIDTracer.h"

using namespace ev3api;
using namespace std;

int sleepDuration = 1000 * 500;

PIDTargetBrightnessCalibrator::PIDTargetBrightnessCalibrator(RobotAPI *robotAPI)
{
    this->robotAPI = robotAPI;
};

PIDTargetBrightnessCalibrator::~PIDTargetBrightnessCalibrator()
{
}

void PIDTargetBrightnessCalibrator::readWhiteFromColorSensor()
{
    readedWhite = true;
    white = robotAPI->getColorSensor()->getBrightness();
}

void PIDTargetBrightnessCalibrator::readBlackFromColorSensor()
{
    readedBlack = true;
    black = robotAPI->getColorSensor()->getBrightness();
}

void PIDTargetBrightnessCalibrator::run(RobotAPI *robotAPI)
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
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readBlackFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
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
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readWhiteFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else
    {
        if (!calibratedPIDTracers)
        {
            calibratedPIDTracers = true;
            for (int i = 0; i < ((int)pidTracers.size()); i++)
            {
                pidTracers[i]->setTargetBrightness((getWhite() + getBlack()) / 2);
            }
        }

        stringstream bs;
        stringstream ws;
        stringstream brightnessStream;

        bs << "black bright:" << getBlack();
        ws << "white bright:" << getWhite();
        brightnessStream << "brightness: " << float(robotAPI->getColorSensor()->getBrightness());

        vector<string> messageLines;
        messageLines.push_back("calibrated!");
        messageLines.push_back(bs.str());
        messageLines.push_back(ws.str());
        messageLines.push_back(brightnessStream.str());
        messageLines.push_back("press touch sensor");
        messageLines.push_back("      to START!");
        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }
}

PIDTargetBrightnessCalibrator *PIDTargetBrightnessCalibrator::generateReverseCommand()
{
    return new PIDTargetBrightnessCalibrator(robotAPI);
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

void PIDTargetBrightnessCalibrator::addPIDTracer(PIDTracer *pidTracer)
{
    pidTracers.push_back(pidTracer);
}