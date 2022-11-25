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

bool equalStr(string str1, string str2)
{
    return str1.compare(str2) == 0;
}

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
        string redCardRVariableName = GET_VARIABLE_NAME(redCardR);
        string redCardGVariableName = GET_VARIABLE_NAME(redCardG);
        string redCardBVariableName = GET_VARIABLE_NAME(redCardB);
        string greenCardRVariableName = GET_VARIABLE_NAME(greenCardR);
        string greenCardGVariableName = GET_VARIABLE_NAME(greenCardG);
        string greenCardBVariableName = GET_VARIABLE_NAME(greenCardB);
        string blueCardRVariableName = GET_VARIABLE_NAME(blueCardR);
        string blueCardGVariableName = GET_VARIABLE_NAME(blueCardG);
        string blueCardBVariableName = GET_VARIABLE_NAME(blueCardB);
        string yellowCardRVariableName = GET_VARIABLE_NAME(yellowCardR);
        string yellowCardGVariableName = GET_VARIABLE_NAME(yellowCardG);
        string yellowCardBVariableName = GET_VARIABLE_NAME(yellowCardB);

        if (equalStr(blackWhiteEdgeTargetBrightnessVariableName, key))
        {
            blackWhiteEdgeTargetBrightness = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeTargetBrightness));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeTargetBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteBrightnessVariableName, key))
        {
            whiteBrightness = value;
            writeDebug(GET_VARIABLE_NAME(whiteBrightness));
            writeDebug(": ");
            writeDebug(whiteBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackBrightnessVariableName, key))
        {
            blackBrightness = value;
            writeDebug(GET_VARIABLE_NAME(blackBrightness));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeTargetBrightness);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteAtSlalomRVariableName, key))
        {
            whiteAtSlalomR = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomR));
            writeDebug(": ");
            writeDebug(whiteAtSlalomR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteAtSlalomGVariableName, key))
        {
            whiteAtSlalomG = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomG));
            writeDebug(": ");
            writeDebug(whiteAtSlalomG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteAtSlalomBVariableName, key))
        {
            whiteAtSlalomB = value;
            writeDebug(GET_VARIABLE_NAME(whiteAtSlalomB));
            writeDebug(": ");
            writeDebug(whiteAtSlalomB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(grayRVariableName, key))
        {
            grayR = value;
            writeDebug(GET_VARIABLE_NAME(grayR));
            writeDebug(": ");
            writeDebug(grayR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(grayGVariableName, key))
        {
            grayG = value;
            writeDebug(GET_VARIABLE_NAME(grayG));
            writeDebug(": ");
            writeDebug(grayG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(grayBVariableName, key))
        {
            grayB = value;
            writeDebug(GET_VARIABLE_NAME(grayB));
            writeDebug(": ");
            writeDebug(grayB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackRVariableName, key))
        {
            blackR = value;
            writeDebug(GET_VARIABLE_NAME(blackR));
            writeDebug(": ");
            writeDebug(blackR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackGVariableName, key))
        {
            blackG = value;
            writeDebug(GET_VARIABLE_NAME(blackG));
            writeDebug(": ");
            writeDebug(blackG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackBVariableName, key))
        {
            blackB = value;
            writeDebug(GET_VARIABLE_NAME(blackB));
            writeDebug(": ");
            writeDebug(blackB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteRVariableName, key))
        {
            whiteR = value;
            writeDebug(GET_VARIABLE_NAME(whiteR));
            writeDebug(": ");
            writeDebug(whiteR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteGVariableName, key))
        {
            whiteG = value;
            writeDebug(GET_VARIABLE_NAME(whiteG));
            writeDebug(": ");
            writeDebug(whiteG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(whiteBVariableName, key))
        {
            whiteB = value;
            writeDebug(GET_VARIABLE_NAME(whiteB));
            writeDebug(": ");
            writeDebug(whiteB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackWhiteEdgeRVariableName, key))
        {
            blackWhiteEdgeR = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeR));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackWhiteEdgeGVariableName, key))
        {
            blackWhiteEdgeG = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeG));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blackWhiteEdgeBVariableName, key))
        {
            blackWhiteEdgeB = value;
            writeDebug(GET_VARIABLE_NAME(blackWhiteEdgeB));
            writeDebug(": ");
            writeDebug(blackWhiteEdgeB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueWhiteEdgeRVariableName, key))
        {
            blueWhiteEdgeR = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeR));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueWhiteEdgeGVariableName, key))
        {
            blueWhiteEdgeG = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeG));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueWhiteEdgeBVariableName, key))
        {
            blueWhiteEdgeB = value;
            writeDebug(GET_VARIABLE_NAME(blueWhiteEdgeB));
            writeDebug(": ");
            writeDebug(blueWhiteEdgeB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(redRVariableName, key))
        {
            redR = value;
            writeDebug(GET_VARIABLE_NAME(redR));
            writeDebug(": ");
            writeDebug(redR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(redGVariableName, key))
        {
            redG = value;
            writeDebug(GET_VARIABLE_NAME(redG));
            writeDebug(": ");
            writeDebug(redG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(redBVariableName, key))
        {
            redB = value;
            writeDebug(GET_VARIABLE_NAME(redB));
            writeDebug(": ");
            writeDebug(redB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(greenRVariableName, key))
        {
            greenR = value;
            writeDebug(GET_VARIABLE_NAME(greenR));
            writeDebug(": ");
            writeDebug(greenR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(greenGVariableName, key))
        {
            greenG = value;
            writeDebug(GET_VARIABLE_NAME(greenG));
            writeDebug(": ");
            writeDebug(greenG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(greenBVariableName, key))
        {
            greenB = value;
            writeDebug(GET_VARIABLE_NAME(greenB));
            writeDebug(": ");
            writeDebug(greenB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueRVariableName, key))
        {
            blueR = value;
            writeDebug(GET_VARIABLE_NAME(blueR));
            writeDebug(": ");
            writeDebug(blueR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueGVariableName, key))
        {
            blueG = value;
            writeDebug(GET_VARIABLE_NAME(blueG));
            writeDebug(": ");
            writeDebug(blueG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueBVariableName, key))
        {
            blueB = value;
            writeDebug(GET_VARIABLE_NAME(blueB));
            writeDebug(": ");
            writeDebug(blueB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(yellowRVariableName, key))
        {
            yellowR = value;
            writeDebug(GET_VARIABLE_NAME(yellowR));
            writeDebug(": ");
            writeDebug(yellowR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(yellowGVariableName, key))
        {
            yellowG = value;
            writeDebug(GET_VARIABLE_NAME(yellowG));
            writeDebug(": ");
            writeDebug(yellowG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(yellowBVariableName, key))
        {
            yellowB = value;
            writeDebug(GET_VARIABLE_NAME(yellowB));
            writeDebug(": ");
            writeDebug(yellowB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(redCardRVariableName, key))
        {
            redCardR = value;
            writeDebug(GET_VARIABLE_NAME(redCardR));
            writeDebug(": ");
            writeDebug(redCardR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(redCardGVariableName, key))
        {
            redCardG = value;
            writeDebug(GET_VARIABLE_NAME(redCardG));
            writeDebug(": ");
            writeDebug(redCardG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(redCardBVariableName, key))
        {
            redCardB = value;
            writeDebug(GET_VARIABLE_NAME(redCardB));
            writeDebug(": ");
            writeDebug(redCardB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(greenCardRVariableName, key))
        {
            greenCardR = value;
            writeDebug(GET_VARIABLE_NAME(greenCardR));
            writeDebug(": ");
            writeDebug(greenCardR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(greenCardGVariableName, key))
        {
            greenCardG = value;
            writeDebug(GET_VARIABLE_NAME(greenCardG));
            writeDebug(": ");
            writeDebug(greenCardG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(greenCardBVariableName, key))
        {
            greenCardB = value;
            writeDebug(GET_VARIABLE_NAME(greenCardB));
            writeDebug(": ");
            writeDebug(greenCardB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueCardRVariableName, key))
        {
            blueCardR = value;
            writeDebug(GET_VARIABLE_NAME(blueCardR));
            writeDebug(": ");
            writeDebug(blueCardR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueCardGVariableName, key))
        {
            blueCardG = value;
            writeDebug(GET_VARIABLE_NAME(blueCardG));
            writeDebug(": ");
            writeDebug(blueCardG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(blueCardBVariableName, key))
        {
            blueCardB = value;
            writeDebug(GET_VARIABLE_NAME(blueCardB));
            writeDebug(": ");
            writeDebug(blueCardB);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(yellowCardRVariableName, key))
        {
            yellowCardR = value;
            writeDebug(GET_VARIABLE_NAME(yellowCardR));
            writeDebug(": ");
            writeDebug(yellowCardR);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(yellowCardGVariableName, key))
        {
            yellowCardG = value;
            writeDebug(GET_VARIABLE_NAME(yellowCardG));
            writeDebug(": ");
            writeDebug(yellowCardG);
            flushDebug(DEBUG, robotAPI);
        }
        else if (equalStr(yellowCardBVariableName, key))
        {
            yellowCardB = value;
            writeDebug(GET_VARIABLE_NAME(yellowCardB));
            writeDebug(": ");
            writeDebug(yellowCardB);
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
    ss << GET_VARIABLE_NAME(redCardR) << "=" << float(redCardR) << endl;
    ss << GET_VARIABLE_NAME(redCardG) << "=" << float(redCardG) << endl;
    ss << GET_VARIABLE_NAME(redCardB) << "=" << float(redCardB) << endl;
    ss << GET_VARIABLE_NAME(greenCardR) << "=" << float(greenCardR) << endl;
    ss << GET_VARIABLE_NAME(greenCardG) << "=" << float(greenCardG) << endl;
    ss << GET_VARIABLE_NAME(greenCardB) << "=" << float(greenCardB) << endl;
    ss << GET_VARIABLE_NAME(blueCardR) << "=" << float(blueCardR) << endl;
    ss << GET_VARIABLE_NAME(blueCardG) << "=" << float(blueCardG) << endl;
    ss << GET_VARIABLE_NAME(blueCardB) << "=" << float(blueCardB) << endl;
    ss << GET_VARIABLE_NAME(yellowCardR) << "=" << float(yellowCardR) << endl;
    ss << GET_VARIABLE_NAME(yellowCardG) << "=" << float(yellowCardG) << endl;
    ss << GET_VARIABLE_NAME(yellowCardB) << "=" << float(yellowCardB) << endl;

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
    writeDebug(GET_VARIABLE_NAME(redCardR));
    writeDebug(": ");
    writeDebug(redCardR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(redCardG));
    writeDebug(": ");
    writeDebug(redCardG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(redCardB));
    writeDebug(": ");
    writeDebug(redCardB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(greenCardR));
    writeDebug(": ");
    writeDebug(greenCardR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(greenCardG));
    writeDebug(": ");
    writeDebug(greenCardG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(greenCardB));
    writeDebug(": ");
    writeDebug(greenCardB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueCardR));
    writeDebug(": ");
    writeDebug(blueCardR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueCardG));
    writeDebug(": ");
    writeDebug(blueCardG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(blueCardB));
    writeDebug(": ");
    writeDebug(blueCardB);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(yellowCardR));
    writeDebug(": ");
    writeDebug(yellowCardR);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(yellowCardG));
    writeDebug(": ");
    writeDebug(yellowCardG);
    flushDebug(DEBUG, robotAPI);
    writeDebug(GET_VARIABLE_NAME(yellowCardB));
    writeDebug(": ");
    writeDebug(yellowCardB);

    FILE *preCalibratedValuesFile = fopen(preCalibratedValuesFileName, "w");
    fprintf(preCalibratedValuesFile, ss.str().c_str());
    fclose(preCalibratedValuesFile);
}

PIDTargetColorBrightnessCalibrator::PIDTargetColorBrightnessCalibrator(BrightnessCalibrateMode brightnessCalibrateMode)
{
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
            stringstream vs;
            vs << "voltage: " << float(ev3_battery_voltage_mV());
            printedReadRedCardMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read red card");
            messageLines.push_back(" from color sensor");
            messageLines.push_back("if down key press then load pre calibrated data");
            messageLines.push_back(vs.str());
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
            printedReadSlalomWhiteColorMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read white at slalom");
            messageLines.push_back(" from color sensor");
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
            printedReadGrayColorMessage = true;
            vector<string> messageLines;
            messageLines.push_back("calibrating");
            messageLines.push_back("press right key");
            messageLines.push_back(" read gray color");
            messageLines.push_back(" from color sensor");
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
    this->robotAPI = robotAPI;
    return;
}

PIDTargetColorBrightnessCalibrator *PIDTargetColorBrightnessCalibrator::generateReverseCommand()
{
    return new PIDTargetColorBrightnessCalibrator(brightnessCalibrateMode);
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