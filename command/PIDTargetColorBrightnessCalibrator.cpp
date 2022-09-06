#include "PIDTargetColorBrightnessCalibrator.h"
#include "PrintMessage.h"
#include "Clock.h"
#include "string"
#include "sstream"
#include "vector"
#include "RobotAPI.h"
#include "PIDTracer.h"
#include "ColorPIDTracer.h"
#include "ColorPIDTracer.h"
#include "Sensor.h"
#include "Setting.h"

using namespace ev3api;
using namespace std;

PIDTargetColorBrightnessCalibrator::PIDTargetColorBrightnessCalibrator(RobotAPI *robotAPI)
{
    this->robotAPI = robotAPI;
};

PIDTargetColorBrightnessCalibrator::~PIDTargetColorBrightnessCalibrator()
{
}

void PIDTargetColorBrightnessCalibrator::readWhiteBrightnessFromColorSensor()
{
    readedWhiteBrightness = true;
    whiteBrightness = robotAPI->getColorSensor()->getBrightness();
}

void PIDTargetColorBrightnessCalibrator::readBlackBrightnessFromColorSensor()
{
    readedBlackBrightness = true;
    blackBrightness = robotAPI->getColorSensor()->getBrightness();
}

void PIDTargetColorBrightnessCalibrator::run(RobotAPI *robotAPI)
{
    int sleepDuration = 1000 * 500;
    if (!readedBlueColor && calibrateBlue)
    {
        if (!printedReadBlueMessage)
        {
            stringstream vs;
            vs << "voltage: " << float(ev3_battery_voltage_mV());

            printedReadBlueMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read blue");
            messageLines.push_back(" from color sensor");
            messageLines.push_back(vs.str());
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            b_r = rawColor.r;
            b_g = rawColor.g;
            b_b = rawColor.b;
            readedBlueColor = true;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!isReadedBlackBrightness())
    {
        if (!printedReadBlackMessage)
        {
            printedReadBlackMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read black");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            d_r = rawColor.r;
            d_g = rawColor.g;
            d_b = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);

            readBlackBrightnessFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
            readBlackColorFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!isReadedWhiteBrightness())
    {
        if (!printedReadWhiteMessage)
        {
            printedReadWhiteMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read white");
            messageLines.push_back(" from color sensor");

            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            w_r = rawColor.r;
            w_g = rawColor.g;
            w_b = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);

            readWhiteBrightnessFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
            readWhiteColorFromColorSensor();
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
                pidTracers[i]->setTargetBrightness((getWhiteBrightness() + getBlackBrightness()) / 2);
            }
            for (int i = 0; i < ((int)colorPIDTracers.size()); i++)
            {
                rgb_raw_t targetRGB;
                targetRGB.r = (getWhiteColor().r + getBlackColor().r) / 2;
                targetRGB.g = (getWhiteColor().g + getBlackColor().g) / 2;
                targetRGB.b = (getWhiteColor().b + getBlackColor().b) / 2;
                colorPIDTracers[i]->setTargetColor(targetRGB);
            }

            bw_r = (b_r + w_r) / 2;
            bw_g = (b_g + w_g) / 2;
            bw_b = (b_b + w_b) / 2;
        }
        stringstream bs;
        stringstream ws;
        stringstream bcs;
        stringstream wcs;
        stringstream ts;
        stringstream trs;
        stringstream brightnessStream;

        bs << "black bright :" << float(getBlackBrightness());
        ws << "white bright :" << float(getWhiteBrightness());
        bcs << "black r:" << float(getBlackColor().r) << " g:" << float(getBlackColor().g) << " b:" << float(getBlackColor().b);
        wcs << "white r:" << float(getWhiteColor().r) << " g:" << float(getWhiteColor().g) << " b:" << float(getWhiteColor().b);
        ts << "target bright:" << float((getWhiteBrightness() + getBlackBrightness()) / 2);
        trs << "target r:" << float((getWhiteColor().r + getBlackColor().r) / 2) << " g:" << float((getWhiteColor().g + getBlackColor().g) / 2) << " b:" << float((getWhiteColor().b + getBlackColor().b) / 2);
        brightnessStream << "brightness: " << float(robotAPI->getColorSensor()->getBrightness());

        vector<string> messageLines;
        messageLines.push_back("calibrated!");
        messageLines.push_back(bs.str());
        messageLines.push_back(ws.str());
        messageLines.push_back(bcs.str());
        messageLines.push_back(wcs.str());
        messageLines.push_back(ts.str());
        messageLines.push_back(trs.str());
        messageLines.push_back(brightnessStream.str());
        messageLines.push_back("press touch sensor");
        messageLines.push_back("      to START!");
        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }
}

void PIDTargetColorBrightnessCalibrator::preparation(RobotAPI *robotAPI)
{
    return;
}

PIDTargetColorBrightnessCalibrator *PIDTargetColorBrightnessCalibrator::generateReverseCommand()
{
    return new PIDTargetColorBrightnessCalibrator(robotAPI);
}

int PIDTargetColorBrightnessCalibrator::getBlackBrightness()
{
    return blackBrightness;
}

int PIDTargetColorBrightnessCalibrator::getWhiteBrightness()
{
    return whiteBrightness;
}

bool PIDTargetColorBrightnessCalibrator::isReadedBlackBrightness()
{
    return readedBlackBrightness;
}

bool PIDTargetColorBrightnessCalibrator::isReadedWhiteBrightness()
{
    return readedWhiteBrightness;
}

void PIDTargetColorBrightnessCalibrator::addPIDTracer(PIDTracer *pidTracer)
{
    pidTracers.push_back(pidTracer);
}

rgb_raw_t PIDTargetColorBrightnessCalibrator::getBlackColor()
{
    return blackColor;
}

rgb_raw_t PIDTargetColorBrightnessCalibrator::getWhiteColor()
{
    return whiteColor;
}

bool PIDTargetColorBrightnessCalibrator::isReadedBlackColor()
{
    return readedBlackColor;
}

bool PIDTargetColorBrightnessCalibrator::isReadedWhiteColor()
{
    return readedWhiteColor;
}

void PIDTargetColorBrightnessCalibrator::readWhiteColorFromColorSensor()
{
    rgb_raw_t whiteTemp;
    robotAPI->getColorSensor()->getRawColor(whiteTemp);
    whiteColor = whiteTemp;
    readedWhiteColor = true;
}

void PIDTargetColorBrightnessCalibrator::readBlackColorFromColorSensor()
{
    rgb_raw_t blackTemp;
    robotAPI->getColorSensor()->getRawColor(blackTemp);
    blackColor = blackTemp;
    readedBlackColor = true;
}

void PIDTargetColorBrightnessCalibrator::addColorPIDTracer(ColorPIDTracer *pidTracer)
{
    colorPIDTracers.push_back(pidTracer);
}