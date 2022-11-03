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

// ファイルからキャリブレーションされた値を読み込む関数。
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

        string blackWhiteEdgeTargetBrightnessVariableName = GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness);
        string whiteBrightnessVariableName = GET_VARIABLE_NAME(whiteBrightness);
        string blackBrightnessVariableName = GET_VARIABLE_NAME(blackBrightness);
        string whiteAtSlalomRVariableName = GET_VARIABLE_NAME(whiteAtSlalomR);
        string whiteAtSlalomGVariableName = GET_VARIABLE_NAME(whiteAtSlalomG);
        string whiteAtSlalomBVariableName = GET_VARIABLE_NAME(whiteAtSlalomB);
        string grayRVariableName = GET_VARIABLE_NAME(grayR);
        string grayGVariableName = GET_VARIABLE_NAME(grayG);
        string grayBVariableName = GET_VARIABLE_NAME(grayB);
        string blackRVariableName = GET_VARIABLE_NAME(blackR);
        string blackGVariableName = GET_VARIABLE_NAME(blackG);
        string blackBVariableName = GET_VARIABLE_NAME(blackB);
        string whiteRVariableName = GET_VARIABLE_NAME(whiteR);
        string whiteGVariableName = GET_VARIABLE_NAME(whiteG);
        string whiteBVariableName = GET_VARIABLE_NAME(whiteB);
        string blackWhiteEdgeRVariableName = GET_VARIABLE_NAME(blackWhiteEdgeR);
        string blackWhiteEdgeGVariableName = GET_VARIABLE_NAME(blackWhiteEdgeG);
        string blackWhiteEdgeBVariableName = GET_VARIABLE_NAME(blackWhiteEdgeB);
        string blueWhiteEdgeRVariableName = GET_VARIABLE_NAME(blueWhiteEdgeR);
        string blueWhiteEdgeGVariableName = GET_VARIABLE_NAME(blueWhiteEdgeG);
        string blueWhiteEdgeBVariableName = GET_VARIABLE_NAME(blueWhiteEdgeB);
        string redRVariableName = GET_VARIABLE_NAME(redR);
        string redGVariableName = GET_VARIABLE_NAME(redG);
        string redBVariableName = GET_VARIABLE_NAME(redB);
        string greenRVariableName = GET_VARIABLE_NAME(greenR);
        string greenGVariableName = GET_VARIABLE_NAME(greenG);
        string greenBVariableName = GET_VARIABLE_NAME(greenB);
        string blueRVariableName = GET_VARIABLE_NAME(blueR);
        string blueGVariableName = GET_VARIABLE_NAME(blueG);
        string blueBVariableName = GET_VARIABLE_NAME(blueB);
        string yellowRVariableName = GET_VARIABLE_NAME(yellowR);
        string yellowGVariableName = GET_VARIABLE_NAME(yellowG);
        string yellowBVariableName = GET_VARIABLE_NAME(yellowB);

        if (equal(blackWhiteEdgeTargetBrightnessVariableName.begin(), blackWhiteEdgeTargetBrightnessVariableName.end(), key.c_str()))
        {
            blackWhiteEdgeTargetBrightness = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeTargetBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        if (equal(whiteBrightnessVariableName.begin(), whiteBrightnessVariableName.end(), key.c_str()))
        {
            whiteBrightness = value;
            writeDebug(GET_VARIABLE_NAME(whiteBrightness));
            writeDebug(": ");
            writeDebug(whiteBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        if (equal(blackBrightnessVariableName.begin(), blackBrightnessVariableName.end(), key.c_str()))
        {
            blackBrightness = value;
            writeDebug(GET_VARIABLE_NAME(blackBrightness));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeTargetBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(whiteAtSlalomRVariableName.begin(), whiteAtSlalomRVariableName.end(), key.c_str()))
        {
            whiteAtSlalomR = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomR));
            writeDebug(": ");
            writeDebug(whiteAtSlalomR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(whiteAtSlalomGVariableName.begin(), whiteAtSlalomGVariableName.end(), key.c_str()))
        {
            whiteAtSlalomG = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomG));
            writeDebug(": ");
            writeDebug(whiteAtSlalomG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(whiteAtSlalomBVariableName.begin(), whiteAtSlalomBVariableName.end(), key.c_str()))
        {
            whiteAtSlalomB = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomB));
            writeDebug(": ");
            writeDebug(whiteAtSlalomB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(grayRVariableName.begin(), grayRVariableName.end(), key.c_str()))
        {
            grayR = value;
            writeDebug(GET_VARIABLE_NAME(grayR));
            writeDebug(": ");
            writeDebug(grayR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(grayGVariableName.begin(), grayGVariableName.end(), key.c_str()))
        {
            grayG = value;
            writeDebug(GET_VARIABLE_NAME(grayG));
            writeDebug(": ");
            writeDebug(grayG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(grayBVariableName.begin(), grayBVariableName.end(), key.c_str()))
        {
            grayB = value;
            writeDebug(GET_VARIABLE_NAME(grayB));
            writeDebug(": ");
            writeDebug(grayB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blackRVariableName.begin(), blackRVariableName.end(), key.c_str()))
        {
            blackR = value;
            writeDebug(GET_VARIABLE_NAME(blackR));
            writeDebug(": ");
            writeDebug(blackR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blackGVariableName.begin(), blackGVariableName.end(), key.c_str()))
        {
            blackG = value;
            writeDebug(GET_VARIABLE_NAME(blackG));
            writeDebug(": ");
            writeDebug(blackG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blackBVariableName.begin(), blackBVariableName.end(), key.c_str()))
        {
            blackB = value;
            writeDebug(GET_VARIABLE_NAME(blackB));
            writeDebug(": ");
            writeDebug(blackB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(whiteRVariableName.begin(), whiteRVariableName.end(), key.c_str()))
        {
            whiteR = value;
            writeDebug(GET_VARIABLE_NAME(whiteR));
            writeDebug(": ");
            writeDebug(whiteR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(whiteGVariableName.begin(), whiteGVariableName.end(), key.c_str()))
        {
            whiteG = value;
            writeDebug(GET_VARIABLE_NAME(whiteG));
            writeDebug(": ");
            writeDebug(whiteG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(whiteBVariableName.begin(), whiteBVariableName.end(), key.c_str()))
        {
            whiteB = value;
            writeDebug(GET_VARIABLE_NAME(whiteB));
            writeDebug(": ");
            writeDebug(whiteB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blackWhiteEdgeRVariableName.begin(), blackWhiteEdgeRVariableName.end(), key.c_str()))
        {
            blackWhiteEdgeR = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeR));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blackWhiteEdgeGVariableName.begin(), blackWhiteEdgeGVariableName.end(), key.c_str()))
        {
            blackWhiteEdgeG = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeG));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blackWhiteEdgeBVariableName.begin(), blackWhiteEdgeBVariableName.end(), key.c_str()))
        {
            blackWhiteEdgeB = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeB));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blueWhiteEdgeRVariableName.begin(), blueWhiteEdgeRVariableName.end(), key.c_str()))
        {
            blueWhiteEdgeR = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeR));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blueWhiteEdgeGVariableName.begin(), blueWhiteEdgeGVariableName.end(), key.c_str()))
        {
            blueWhiteEdgeG = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeG));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blueWhiteEdgeBVariableName.begin(), blueWhiteEdgeBVariableName.end(), key.c_str()))
        {
            blueWhiteEdgeB = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeB));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(redRVariableName.begin(), redRVariableName.begin(), key.c_str()))
        {
            redR = value;
            writeDebug(GET_VARIABLE_NAME(redR));
            writeDebug(": ");
            writeDebug(redR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(redGVariableName.begin(), redGVariableName.begin(), key.c_str()))
        {
            redG = value;
            writeDebug(GET_VARIABLE_NAME(redG));
            writeDebug(": ");
            writeDebug(redG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(redBVariableName.begin(), redBVariableName.begin(), key.c_str()))
        {
            redB = value;
            writeDebug(GET_VARIABLE_NAME(redB));
            writeDebug(": ");
            writeDebug(redB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(greenRVariableName.begin(), greenRVariableName.end(), key.c_str()))
        {
            greenR = value;
            writeDebug(GET_VARIABLE_NAME(greenR));
            writeDebug(": ");
            writeDebug(greenR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(greenGVariableName.begin(), greenGVariableName.end(), key.c_str()))
        {
            greenG = value;
            writeDebug(GET_VARIABLE_NAME(greenG));
            writeDebug(": ");
            writeDebug(greenG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(greenBVariableName.begin(), greenBVariableName.end(), key.c_str()))
        {
            greenB = value;
            writeDebug(GET_VARIABLE_NAME(greenB));
            writeDebug(": ");
            writeDebug(greenB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blueRVariableName.begin(), blueRVariableName.end(), key.c_str()))
        {
            blueR = value;
            writeDebug(GET_VARIABLE_NAME(blueR));
            writeDebug(": ");
            writeDebug(blueR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blueGVariableName.begin(), blueGVariableName.end(), key.c_str()))
        {
            blueG = value;
            writeDebug(GET_VARIABLE_NAME(blueG));
            writeDebug(": ");
            writeDebug(blueG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(blueBVariableName.begin(), blueBVariableName.end(), key.c_str()))
        {
            blueB = value;
            writeDebug(GET_VARIABLE_NAME(blueB));
            writeDebug(": ");
            writeDebug(blueB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(yellowRVariableName.begin(), yellowRVariableName.end(), key.c_str()))
        {
            yellowR = value;
            writeDebug(GET_VARIABLE_NAME(yellowR));
            writeDebug(": ");
            writeDebug(yellowR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(yellowGVariableName.begin(), yellowGVariableName.end(), key.c_str()))
        {
            yellowG = value;
            writeDebug(GET_VARIABLE_NAME(yellowG));
            writeDebug(": ");
            writeDebug(yellowG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equal(yellowBVariableName.begin(), yellowBVariableName.end(), key.c_str()))
        {
            yellowB = value;
            writeDebug(GET_VARIABLE_NAME(yellowB));
            writeDebug(": ");
            writeDebug(yellowB);
            flushDebug(DEBUG, robotAPI);
        }
    }
}

// ファイルへとキャリブレーションされた値を書き込む関数。
void writeCalibratedValuesToFile(RobotAPI *robotAPI)
{
    stringstream ss;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness) << "=" << float(blackWhiteEdgeTargetBrightness) << endl;
    ss << GET_VARIABLE_NAME(whiteBrightness) << "=" << float(whiteBrightness) << endl;
    ss << GET_VARIABLE_NAME(blackBrightness) << "=" << float(blackBrightness) << endl;
    ss << GET_VARIABLE_NAME(whiteAtSlalomR) << "=" << float(whiteAtSlalomR) << endl;
    ss << GET_VARIABLE_NAME(whiteAtSlalomG) << "=" << float(whiteAtSlalomG) << endl;
    ss << GET_VARIABLE_NAME(whiteAtSlalomB) << "=" << float(whiteAtSlalomB) << endl;
    ss << GET_VARIABLE_NAME(grayR) << "=" << float(grayR) << endl;
    ss << GET_VARIABLE_NAME(grayG) << "=" << float(grayG) << endl;
    ss << GET_VARIABLE_NAME(grayB) << "=" << float(grayB) << endl;
    ss << GET_VARIABLE_NAME(blackR) << "=" << float(blackR) << endl;
    ss << GET_VARIABLE_NAME(blackG) << "=" << float(blackG) << endl;
    ss << GET_VARIABLE_NAME(blackB) << "=" << float(blackB) << endl;
    ss << GET_VARIABLE_NAME(whiteR) << "=" << float(whiteR) << endl;
    ss << GET_VARIABLE_NAME(whiteG) << "=" << float(whiteG) << endl;
    ss << GET_VARIABLE_NAME(whiteB) << "=" << float(whiteB) << endl;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeR) << "=" << float(blackWhiteEdgeR) << endl;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeG) << "=" << float(blackWhiteEdgeG) << endl;
    ss << GET_VARIABLE_NAME(blackWhiteEdgeB) << "=" << float(blackWhiteEdgeB) << endl;
    ss << GET_VARIABLE_NAME(blueWhiteEdgeR) << "=" << float(blueWhiteEdgeR) << endl;
    ss << GET_VARIABLE_NAME(blueWhiteEdgeG) << "=" << float(blueWhiteEdgeG) << endl;
    ss << GET_VARIABLE_NAME(blueWhiteEdgeB) << "=" << float(blueWhiteEdgeB) << endl;
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
        readedBlueEdgeColor = true;
        readedSlalomWhiteColor = true;
        readedBlackWhiteEdgeColor = true;
        readedGrayColor = true;
        readedRedColor = true;
        readedGreenColor = true;
        readedBlueColor = true;
        readedYellowColor = true;
        readedRedCardColor = true;
        readedGreenCardColor = true;
        readedBlueCardColor = true;
        readedYellowCardColor = true;

        vector<string> messageLines;
        messageLines.push_back("loaded calibrated data from file");
        messageLines.push_back("press right key");
        messageLines.push_back(" reset api");
        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }

    if (ev3_button_is_pressed(ENTER_BUTTON))
    {
        readedBlackWhiteEdgeBrightness = true;
        readedBlackWhiteEdge = true;
        readedWhiteBrightness = true;
        readedBlackBrightness = true;
        readedWhiteColor = true;
        readedBlackColor = true;
        readedBlueEdgeColor = true;
        readedSlalomWhiteColor = true;
        readedBlackWhiteEdgeColor = true;
        readedGrayColor = true;
        readedRedColor = true;
        readedGreenColor = true;
        readedBlueColor = true;
        readedYellowColor = true;
        readedYellowColor = true;
        readedRedCardColor = true;
        readedGreenCardColor = true;
        readedBlueCardColor = true;
        readedYellowCardColor = true;

        vector<string> messageLines;
        messageLines.push_back("use default value.");
        messageLines.push_back("press right key");
        messageLines.push_back(" reset api");
        PrintMessage printMessage(messageLines, true);
        printMessage.run(robotAPI);
    }

    if (!readedRedCardColor && calibrateRedCard)
    {
        if (!printedReadRedCardMessage)
        {
            printedReadRedCardMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read red card");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedRedCardColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            redCardR = rawColor.r;
            redCardG = rawColor.g;
            redCardB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedGreenCardColor && calibrateGreenCard)
    {
        if (!printedReadGreenCardMessage)
        {
            printedReadGreenCardMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read green card");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedGreenCardColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            greenCardR = rawColor.r;
            greenCardG = rawColor.g;
            greenCardB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedBlueCardColor && calibrateBlueCard)
    {
        if (!printedReadBlueCardMessage)
        {
            printedReadBlueCardMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read blue card");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedBlueCardColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            blueCardR = rawColor.r;
            blueCardG = rawColor.g;
            blueCardB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedYellowCardColor && calibrateYellowCard)
    {
        if (!printedReadYellowCardMessage)
        {
            printedReadYellowCardMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read yellow card");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedYellowCardColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            yellowCardR = rawColor.r;
            yellowCardG = rawColor.g;
            yellowCardB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }

    else if (!readedSlalomWhiteColor && calibrateWhiteAtSlalom)
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
            blueWhiteEdgeR = rawColor.r;
            blueWhiteEdgeG = rawColor.g;
            blueWhiteEdgeB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }

    else if (!readedRedColor && calibrateRed)
    {
        if (!printedReadRedMessage)
        {
            printedReadRedMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read red");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedRedColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            redR = rawColor.r;
            redG = rawColor.g;
            redB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedGreenColor && calibrateGreen)
    {
        if (!printedReadGreenMessage)
        {
            printedReadGreenMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read green");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedGreenColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            greenR = rawColor.r;
            greenG = rawColor.g;
            greenB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedBlueColor && calibrateBlue)
    {
        if (!printedReadBlueMessage)
        {
            printedReadBlueMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read blue");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedBlueColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            blueR = rawColor.r;
            blueG = rawColor.g;
            blueB = rawColor.b;
            robotAPI->getClock()->sleep(sleepDuration);
        }
    }
    else if (!readedYellowColor && calibrateYellow)
    {
        if (!printedReadYellowMessage)
        {
            printedReadYellowMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read yellow");
            messageLines.push_back(" from color sensor");
            PrintMessage printMessage(messageLines, true);
            printMessage.run(robotAPI);
        }
        if (ev3_button_is_pressed(RIGHT_BUTTON))
        {
            readedYellowColor = true;
            rgb_raw_t rawColor;
            robotAPI->getColorSensor()->getRawColor(rawColor);
            yellowR = rawColor.r;
            yellowG = rawColor.g;
            yellowB = rawColor.b;
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
                    break;
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

vector<PIDTracer *> *PIDTargetColorBrightnessCalibrator::getPIDTracers()
{
    vector<PIDTracer *> *result = new vector<PIDTracer *>();
    for (int i = 0; i < ((int)pidTracers.size()); i++)
    {
        result->push_back(pidTracers[i]);
    }
    return result;
}

vector<ColorPIDTracer *> *PIDTargetColorBrightnessCalibrator::getColorPIDTracers()
{
    vector<ColorPIDTracer *> *result = new vector<ColorPIDTracer *>();
    for (int i = 0; i < ((int)colorPIDTracers.size()); i++)
    {
        result->push_back(colorPIDTracers[i]);
    }
    return result;
}