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
    whiteBrightness = robotAPI->getColorSensor()->getBrightness();
    readedWhiteBrightness = true;
}

void PIDTargetColorBrightnessCalibrator::readBlackBrightnessFromColorSensor()
{
    blackBrightness = robotAPI->getColorSensor()->getBrightness();
    readedBlackBrightness = true;
}

void PIDTargetColorBrightnessCalibrator::run(RobotAPI *robotAPI)
{
    int sleepDuration = 1000 * 1000;
    if (!readedSlalomWhiteColor && calibrateSlalomWhite)
    {
        if (!printedReadSlalomWhiteColorMessage)
        {
            stringstream vs;
            vs << "voltage: " << float(ev3_battery_voltage_mV());

            printedReadSlalomWhiteColorMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read white at slalom");
            messageLines.push_back(" from color sensor");
            messageLines.push_back(vs.str());
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            sw_r = rawColor.r;
            sw_g = rawColor.g;
            sw_b = rawColor.b;
            readedSlalomWhiteColor = true;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedBlueEdgeColor && calibrateBlueEdge)
    {
        if (!printedReadBlueEdgeMessage)
        {
            printedReadBlueEdgeMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read blue white edge");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            bw_r = rawColor.r;
            bw_g = rawColor.g;
            bw_b = rawColor.b;
            readedBlueEdgeColor = true;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedBlueColor && calibrateBlue)
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
    else if (!isReadedBlackBrightness() && calibrateBlack)
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
    else if ((!isReadedBlackWhiteEdgeBrightness() || !isReadedBlackWhiteEdgeColor()) && calibrateBlackWhiteEdge)
    {
        if (!printedReadBlackWhiteEdgeMessage)
        {
            printedReadBlackWhiteEdgeMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read black white edge");
            messageLines.push_back(" from color sensor");

            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            dw_r = rawColor.r;
            dw_g = rawColor.g;
            dw_b = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);

            readBlackWhiteEdgeBrightnessFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
            readBlackWhiteEdgeColorFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!isReadedWhiteBrightness() && calibrateWhite)
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
    else if (!isResetedAPI())
    {
        if (!printedResetAPIMessage)
        {
            printedResetAPIMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" reset api");

            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            robotAPI->getClock()->sleep(sleepDuration);
            resetAPI();
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
                pidTracers[i]->setTargetBrightness(blackWhiteEdgeBrightness);
            }
            for (int i = 0; i < ((int)colorPIDTracers.size()); i++)
            {
                colorPIDTracers[i]->setTargetColor(blackWhiteEdgeColor);
            }
        }
        stringstream bws;
        stringstream bwcs;
        stringstream brightnessStream;

        bws << "edge bright :" << float(blackWhiteEdgeBrightness);
        bwcs << "edge r:" << float(blackWhiteEdgeColor.r) << " g:" << float(blackWhiteEdgeColor.g) << " b:" << float(blackWhiteEdgeColor.b);
        brightnessStream << "brightness: " << float(robotAPI->getColorSensor()->getBrightness());

        vector<string> messageLines;
        messageLines.push_back("calibrated!");
        messageLines.push_back(bws.str());
        messageLines.push_back(bwcs.str());
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

bool PIDTargetColorBrightnessCalibrator::isResetedAPI()
{
    return resetedAPI;
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

void PIDTargetColorBrightnessCalibrator::resetAPI()
{
    robotAPI->reset();
    resetedAPI = true;
}

void PIDTargetColorBrightnessCalibrator::addColorPIDTracer(ColorPIDTracer *pidTracer)
{
    colorPIDTracers.push_back(pidTracer);
}

void PIDTargetColorBrightnessCalibrator::readBlackWhiteEdgeBrightnessFromColorSensor()
{
    blackWhiteEdgeBrightness = robotAPI->getColorSensor()->getBrightness();
    readedBlackWhiteEdgeBrightness = true;
}

void PIDTargetColorBrightnessCalibrator::readBlackWhiteEdgeColorFromColorSensor()
{
    rgb_raw_t blackWhiteEdgeTemp;
    robotAPI->getColorSensor()->getRawColor(blackWhiteEdgeTemp);
    blackWhiteEdgeColor = blackWhiteEdgeTemp;
    readedBlackWhiteEdgeColor = true;
}

bool PIDTargetColorBrightnessCalibrator::isReadedBlackWhiteEdgeColor()
{
    return readedBlackWhiteEdgeColor;
}

bool PIDTargetColorBrightnessCalibrator::isReadedBlackWhiteEdgeBrightness()
{
    return readedBlackWhiteEdgeBrightness;
}