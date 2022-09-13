#include "PIDTargetColorBrightnessCalibrator.h"
#include "PrintMessage.h"
#include "Clock.h"
#include "string"
#include "vector"
#include "sstream"
#include "vector"
#include "RobotAPI.h"
#include "PIDTracer.h"
#include "ColorPIDTracer.h"
#include "ColorPIDTracer.h"
#include "Sensor.h"
#include "Setting.h"
#include "fstream"
#include "DebugUtil.h"
#include "Util.h"

using namespace ev3api;
using namespace std;

void loadPreCalibratedValuesFromFile(RobotAPI *robotAPI)
{
    fstream fs(preCalibratedValuesFileName);
    string line = "";
    while (getline(fs, line))
    {
        vector<string> keyAndValue = split(line, '=');
        string key = keyAndValue[0];
        int value = 0;
        string valueStr = keyAndValue[1];
        sscanf(valueStr.c_str(), "%d", &value);
        if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness)))
        {
            blackWhiteEdgeTargetBrightness = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeTargetBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteBrightness)))
        {
            whiteBrightness = value;
            writeDebug(GET_VARIABLE_NAME(whiteBrightness));
            writeDebug(": ");
            writeDebug(whiteBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackBrightness)))
        {
            blackBrightness = value;
            writeDebug(GET_VARIABLE_NAME(blackBrightness));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeTargetBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteR)))
        {
            whiteR = value;
            writeDebug(GET_VARIABLE_NAME(whiteR));
            writeDebug(": ");
            writeDebug(whiteR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteG)))
        {
            whiteG = value;
            writeDebug(GET_VARIABLE_NAME(whiteG));
            writeDebug(": ");
            writeDebug(whiteG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteB)))
        {
            whiteB = value;
            writeDebug(GET_VARIABLE_NAME(whiteB));
            writeDebug(": ");
            writeDebug(whiteB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackWhiteEdgeR)))
        {
            blackWhiteEdgeR = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeR));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackWhiteEdgeG)))
        {
            blackWhiteEdgeG = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeG));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackWhiteEdgeB)))
        {
            blackWhiteEdgeB = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeB));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteAtSlalomR)))
        {
            whiteAtSlalomR = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomR));
            writeDebug(": ");
            writeDebug(whiteAtSlalomR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteAtSlalomG)))
        {
            whiteAtSlalomG = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomG));
            writeDebug(": ");
            writeDebug(whiteAtSlalomG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(whiteAtSlalomB)))
        {
            whiteAtSlalomB = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomB));
            writeDebug(": ");
            writeDebug(whiteAtSlalomB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackR)))
        {
            blackR = value;
            writeDebug(GET_VARIABLE_NAME(blackR));
            writeDebug(": ");
            writeDebug(blackR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackG)))
        {
            blackG = value;
            writeDebug(GET_VARIABLE_NAME(blackG));
            writeDebug(": ");
            writeDebug(blackG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blackB)))
        {
            blackB = value;
            writeDebug(GET_VARIABLE_NAME(blackB));
            writeDebug(": ");
            writeDebug(blackB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(redR)))
        {
            redR = value;
            writeDebug(GET_VARIABLE_NAME(redR));
            writeDebug(": ");
            writeDebug(redR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(redG)))
        {
            redG = value;
            writeDebug(GET_VARIABLE_NAME(redG));
            writeDebug(": ");
            writeDebug(redG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(redB)))
        {
            redB = value;
            writeDebug(GET_VARIABLE_NAME(redB));
            writeDebug(": ");
            writeDebug(redB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(greenR)))
        {
            greenR = value;
            writeDebug(GET_VARIABLE_NAME(greenR));
            writeDebug(": ");
            writeDebug(greenR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(greenG)))
        {
            greenG = value;
            writeDebug(GET_VARIABLE_NAME(greenG));
            writeDebug(": ");
            writeDebug(greenG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(greenB)))
        {
            greenB = value;
            writeDebug(GET_VARIABLE_NAME(greenB));
            writeDebug(": ");
            writeDebug(greenB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blueR)))
        {
            blueR = value;
            writeDebug(GET_VARIABLE_NAME(blueR));
            writeDebug(": ");
            writeDebug(blueR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blueG)))
        {
            blueG = value;
            writeDebug(GET_VARIABLE_NAME(blueG));
            writeDebug(": ");
            writeDebug(blueG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blueB)))
        {
            blueB = value;
            writeDebug(GET_VARIABLE_NAME(blueB));
            writeDebug(": ");
            writeDebug(blueB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(yellowR)))
        {
            yellowR = value;
            writeDebug(GET_VARIABLE_NAME(yellowR));
            writeDebug(": ");
            writeDebug(yellowR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(yellowG)))
        {
            yellowG = value;
            writeDebug(GET_VARIABLE_NAME(yellowG));
            writeDebug(": ");
            writeDebug(yellowG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(yellowB)))
        {
            yellowB = value;
            writeDebug(GET_VARIABLE_NAME(yellowB));
            writeDebug(": ");
            writeDebug(yellowB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(grayR)))
        {
            grayR = value;
            writeDebug(GET_VARIABLE_NAME(grayR));
            writeDebug(": ");
            writeDebug(grayR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(grayG)))
        {
            grayG = value;
            writeDebug(GET_VARIABLE_NAME(grayG));
            writeDebug(": ");
            writeDebug(grayG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(grayB)))
        {
            grayB = value;
            writeDebug(GET_VARIABLE_NAME(grayB));
            writeDebug(": ");
            writeDebug(grayB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blueWhiteEdgeR)))
        {
            blueWhiteEdgeR = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeR));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blueWhiteEdgeG)))
        {
            blueWhiteEdgeG = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeG));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(key.begin(), key.end(), GET_VARIABLE_NAME(blueWhiteEdgeB)))
        {
            blueWhiteEdgeB = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeB));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeB);
            flushDebug(DEBUG, robotAPI);
        }
    }
}

void writeCalibratedValuesToFile(RobotAPI *robotAPI)
{
    stringstream ss;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness) << "=" << float(blackWhiteEdgeTargetBrightness) << endl;
    ss << GET_VARIABLE_NAME(whiteBrightness) << "=" << float(whiteBrightness) << endl;
    ss << GET_VARIABLE_NAME(blackBrightness) << "=" << float(blackBrightness) << endl;
    ss << GET_VARIABLE_NAME(whiteR) << "=" << float(whiteR) << endl;
    ss << GET_VARIABLE_NAME(whiteG) << "=" << float(whiteG) << endl;
    ss << GET_VARIABLE_NAME(whiteB) << "=" << float(whiteB) << endl;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeR) << "=" << float(blackWhiteEdgeR) << endl;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeG) << "=" << float(blackWhiteEdgeG) << endl;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeB) << "=" << float(blackWhiteEdgeB) << endl;
    ss << GET_VARIABLE_NAME(whiteAtSlalomR) << "=" << float(whiteAtSlalomR) << endl;
    ss << GET_VARIABLE_NAME(whiteAtSlalomG) << "=" << float(whiteAtSlalomG) << endl;
    ss << GET_VARIABLE_NAME(whiteAtSlalomB) << "=" << float(whiteAtSlalomB) << endl;
    ss << GET_VARIABLE_NAME(blackR) << "=" << float(blackR) << endl;
    ss << GET_VARIABLE_NAME(blackG) << "=" << float(blackG) << endl;
    ss << GET_VARIABLE_NAME(blackB) << "=" << float(blackB) << endl;
    ss << GET_VARIABLE_NAME(redR) << "=" << float(redR) << endl;
    ss << GET_VARIABLE_NAME(redG) << "=" << float(redG) << endl;
    ss << GET_VARIABLE_NAME(redB) << "=" << float(redB) << endl;
    ss << GET_VARIABLE_NAME(greenR) << "=" << float(greenR) << endl;
    ss << GET_VARIABLE_NAME(greenG) << "=" << float(greenG) << endl;
    ss << GET_VARIABLE_NAME(greenB) << "=" << float(greenB) << endl;
    ss << GET_VARIABLE_NAME(blueR) << "=" << float(blueR) << endl;
    ss << GET_VARIABLE_NAME(blueG) << "=" << float(blueG) << endl;
    ss << GET_VARIABLE_NAME(blueB) << "=" << float(blueB) << endl;
    ss << GET_VARIABLE_NAME(yellowR) << "=" << float(yellowR) << endl;
    ss << GET_VARIABLE_NAME(yellowG) << "=" << float(yellowG) << endl;
    ss << GET_VARIABLE_NAME(yellowB) << "=" << float(yellowB) << endl;
    ss << GET_VARIABLE_NAME(grayR) << "=" << float(grayR) << endl;
    ss << GET_VARIABLE_NAME(grayG) << "=" << float(grayG) << endl;
    ss << GET_VARIABLE_NAME(grayB) << "=" << float(grayB) << endl;
    ss << GET_VARIABLE_NAME(blueWhiteEdgeR) << "=" << float(blueWhiteEdgeR) << endl;
    ss << GET_VARIABLE_NAME(blueWhiteEdgeG) << "=" << float(blueWhiteEdgeG) << endl;
    ss << GET_VARIABLE_NAME(blueWhiteEdgeB) << "=" << float(blueWhiteEdgeB) << endl;

    writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness));
    writeDebug(": ");
    writeDebug(blackWhiteEdgeTargetBrightness);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteBrightness));
    writeDebug(": ");
    writeDebug(whiteBrightness);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackBrightness));
    writeDebug(": ");
    writeDebug(blackWhiteEdgeTargetBrightness);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteR));
    writeDebug(": ");
    writeDebug(whiteR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteG));
    writeDebug(": ");
    writeDebug(whiteG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteB));
    writeDebug(": ");
    writeDebug(whiteB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeR));
    writeDebug(": ");
    writeDebug(blackWhiteEdgeR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeG));
    writeDebug(": ");
    writeDebug(blackWhiteEdgeG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeB));
    writeDebug(": ");
    writeDebug(blackWhiteEdgeB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteAtSlalomR));
    writeDebug(": ");
    writeDebug(whiteAtSlalomR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteAtSlalomG));
    writeDebug(": ");
    writeDebug(whiteAtSlalomG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(whiteAtSlalomB));
    writeDebug(": ");
    writeDebug(whiteAtSlalomB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackR));
    writeDebug(": ");
    writeDebug(blackR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackG));
    writeDebug(": ");
    writeDebug(blackG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blackB));
    writeDebug(": ");
    writeDebug(blackB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(redR));
    writeDebug(": ");
    writeDebug(redR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(redG));
    writeDebug(": ");
    writeDebug(redG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(redB));
    writeDebug(": ");
    writeDebug(redB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(greenR));
    writeDebug(": ");
    writeDebug(greenR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(greenG));
    writeDebug(": ");
    writeDebug(greenG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(greenB));
    writeDebug(": ");
    writeDebug(greenB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueR));
    writeDebug(": ");
    writeDebug(blueR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueG));
    writeDebug(": ");
    writeDebug(blueG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueB));
    writeDebug(": ");
    writeDebug(blueB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(yellowR));
    writeDebug(": ");
    writeDebug(yellowR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(yellowG));
    writeDebug(": ");
    writeDebug(yellowG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(yellowB));
    writeDebug(": ");
    writeDebug(yellowB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(grayR));
    writeDebug(": ");
    writeDebug(grayR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(grayG));
    writeDebug(": ");
    writeDebug(grayG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(grayB));
    writeDebug(": ");
    writeDebug(grayB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeR));
    writeDebug(": ");
    writeDebug(blueWhiteEdgeR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeG));
    writeDebug(": ");
    writeDebug(blueWhiteEdgeG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeB));
    writeDebug(": ");
    writeDebug(blueWhiteEdgeB);
    flushDebug(DEBUG, robotAPI);

    FILE *preCalibratedValuesFile = fopen(preCalibratedValuesFileName, "w");
    fprintf(preCalibratedValuesFile, ss.str().c_str());
    fclose(preCalibratedValuesFile);
}

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

    if (ev3_button_is_pressed(DOWN_BUTTON))
    {
        loadPreCalibratedValuesFromFile(robotAPI);

        readedBlackWhiteEdgeBrightness = true;
        readedBlackWhiteEdge = true;
        readedWhiteBrightness = true;
        readedBlackBrightness = true;
        readedWhiteColor = true;
        readedBlackColor = true;
        readedBlueColor = true;
        readedBlueEdgeColor = true;
        readedSlalomWhiteColor = true;
        readedBlackWhiteEdgeColor = true;
        readedGrayColor = true;

        vector<string> messageLines;
        messageLines.push_back("loaded calibrated data from file");
        messageLines.push_back("press right key");
        messageLines.push_back(" reset api");
        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }

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
            messageLines.push_back("if down key press then load pre calibrated data");
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
            printMessageResetting.run(robotAPI);
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
                    pidTracers[i]->setTargetBrightness(blackWhiteEdgeTargetBrightness);
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
            rgb_raw_t blackWhiteEdgeColor;
            blackWhiteEdgeColor.r = blackWhiteEdgeR;
            blackWhiteEdgeColor.g = blackWhiteEdgeG;
            blackWhiteEdgeColor.b = blackWhiteEdgeB;
            for (int i = 0; i < ((int)colorPIDTracers.size()); i++)
            {
                colorPIDTracers[i]->setTargetColor(blackWhiteEdgeColor);
            }
        }

        if (!wrotedToFile && ev3_button_is_pressed(UP_BUTTON))
        {
            wrotedToFile = true;
            writeCalibratedValuesToFile(robotAPI);
        }

        stringstream bws;
        stringstream bwcs;
        stringstream brightnessStream;
        stringstream ups;

        switch (brightnessCalibrateMode)
        {
        case BCM_BlackWhiteEdge:
        {
            bws << "edge bright :" << float(blackWhiteEdgeTargetBrightness);
        }
        case BCM_BlackWhiteAverage:
        {
            bws << "average bright :" << float((whiteBrightness + blackBrightness) / 2);
        }
        }
        bwcs << "edge r:" << float(blackWhiteEdgeR) << " g:" << float(blackWhiteEdgeG) << " b:" << float(blackWhiteEdgeB);
        brightnessStream << "brightness: " << float(robotAPI->getColorSensor()->getBrightness());
        if (!wrotedToFile)
        {
            ups << "up key press save caliburated data";
        }
        else
        {
            ups << "saved calibrated data";
        }

        vector<string> messageLines;
        messageLines.push_back("calibrated!");
        messageLines.push_back(bws.str());
        messageLines.push_back(bwcs.str());
        messageLines.push_back(brightnessStream.str());
        messageLines.push_back(ups.str());
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
    rgb_raw_t white;
    robotAPI->getColorSensor()->getRawColor(white);
    whiteR = white.r;
    whiteG = white.g;
    whiteB = white.b;
    readedWhiteColor = true;
}

void PIDTargetColorBrightnessCalibrator::readBlackColorFromColorSensor()
{
    rgb_raw_t black;
    robotAPI->getColorSensor()->getRawColor(black);
    blackR = black.r;
    blackG = black.g;
    blackB = black.b;
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
    blackWhiteEdgeTargetBrightness = robotAPI->getColorSensor()->getBrightness();
    readedBlackWhiteEdgeBrightness = true;
}

void PIDTargetColorBrightnessCalibrator::readBlackWhiteEdgeColorFromColorSensor()
{
    rgb_raw_t blackWhiteEdge;
    robotAPI->getColorSensor()->getRawColor(blackWhiteEdge);
    blackWhiteEdgeR = blackWhiteEdge.r;
    blackWhiteEdgeG = blackWhiteEdge.g;
    blackWhiteEdgeB = blackWhiteEdge.b;
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