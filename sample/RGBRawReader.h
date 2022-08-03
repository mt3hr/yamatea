#ifndef RGBRawReader_H
#define RGBRawReader_H

#include "ColorSensor.h"
#include "Command.h"

using namespace ev3api;

// RawRGBReader
// 右ボタンでRawRGBを測定するクラス。
// 試走会などで値を取るためのもの
//
// 実方
class RGBRawReader : public Command
{
private:
    ColorSensor *colorSensor;
    rgb_raw_t rgbRaw;
    bool gotRGB = false;

public:
    RGBRawReader(ColorSensor *colorSensor);
    void run();
    Command *generateReverseCommand();
};

#endif