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

PIDTargetColorBrightnessCalibrator::PIDTargetColorBrightnessCalibrator(RobotAPI *robotAPI, BrightnessCalibrateMode brightnessCalibrateMode)
{
    this->robotAPI = robotAPI;
    this->brightnessCalibrateMode = brightnessCalibrateMode;
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
    if (!readedSlalomWhiteColor && calibrateWhiteAtSlalom)
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
            whiteAtSlalomR = rawColor.r;
            whiteAtSlalomG = rawColor.g;
            whiteAtSlalomB = rawColor.b;
            readedSlalomWhiteColor = true;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedGrayColor && calibrateGray)
    {
        if (!printedReadGrayColorMessage)
        {
            stringstream vs;
            vs << "voltage: " << float(ev3_battery_voltage_mV());

            printedReadGrayColorMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read gray color");
            messageLines.push_back(" from color sensor");
            messageLines.push_back(vs.str());
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            grayR = rawColor.r;
            grayG = rawColor.g;
            grayB = rawColor.b;
            readedGrayColor = true;
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
            blackR = rawColor.r;
            blackG = rawColor.g;
            blackB = rawColor.b;
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
            blackR = rawColor.r;
            blackG = rawColor.g;
            blackB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);

            readBlackBrightnessFromColorSensor();
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
            blackWhiteEdgeR = rawColor.r;
            blackWhiteEdgeG = rawColor.g;
            blackWhiteEdgeB = rawColor.b;
            readBlackWhiteEdgeColorFromColorSensor();
            readBlackWhiteEdgeBrightnessFromColorSensor();
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedBlueEdgeColor && calibrateBlueWhiteEdge)
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
            readedBlueEdgeColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            blackWhiteEdgeR = rawColor.r;
            blackWhiteEdgeG = rawColor.g;
            blackWhiteEdgeB = rawColor.b;
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
            whiteR = rawColor.r;
            whiteG = rawColor.g;
            whiteB = rawColor.b;

            readWhiteBrightnessFromColorSensor();
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
            vector<string> messageLines3;
            messageLines3.push_back("reset api");
            messageLines3.push_back("3");
            PrintMessage printMessage3(messageLines3, true);
            printMessage3.run(robotAPI);
            robotAPI->getClock()->sleep(1000 * 1000);
            vector<string> messageLines2;
            messageLines2.push_back("reset api");
            messageLines2.push_back("2");
            PrintMessage printMessage2(messageLines2, true);
            printMessage2.run(robotAPI);
            robotAPI->getClock()->sleep(1000 * 1000);
            vector<string> messageLines1;
            messageLines1.push_back("reset api");
            messageLines1.push_back("1");
            PrintMessage printMessage1(messageLines1, true);
            printMessage1.run(robotAPI);
            robotAPI->getClock()->sleep(1000 * 1000);
            vector<string> messageLinesResetting;
            messageLinesResetting.push_back("reset api");
            messageLinesResetting.push_back("resetting...");
            PrintMessage printMessageResetting(messageLinesResetting, true);
            printMessage1.run(robotAPI);
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
                switch (brightnessCalibrateMode)
                {
                case BCM_BlackWhiteEdge:
                {
                    pidTracers[i]->setTargetBrightness(blackWhiteEdgeBrightness);
                    break;
                }
                case BCM_BlackWhiteAverage:
                {
                    pidTracers[i]->setTargetBrightness((whiteBrightness + blackBrightness) / 2);
                }
                default:
                    break;
                }
            }
            for (int i = 0; i < ((int)colorPIDTracers.size()); i++)
            {
                colorPIDTracers[i]->setTargetColor(blackWhiteEdgeColor);
            }
        }
        stringstream bws;
        stringstream bwcs;
        stringstream brightnessStream;

        switch (brightnessCalibrateMode)
        {
        case BCM_BlackWhiteEdge:
        {
            bws << "edge bright :" << float(blackWhiteEdgeBrightness);
        }
        case BCM_BlackWhiteAverage:
        {
            bws << "average bright :" << float((whiteBrightness + blackBrightness) / 2);
        }
        }
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
    return new PIDTargetColorBrightnessCalibrator(robotAPI, brightnessCalibrateMode);
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