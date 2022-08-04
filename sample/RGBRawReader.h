#ifndef RGBRawReader_H
#define RGBRawReader_H

#include "ColorSensor.h"
#include "Command.h"

using namespace ev3api;

// RawRGBReader
// RawRGBを測定するクラス。右ボタンで取得値記録（ディスプレイに表示）
// 試走会などで値を取るためのもの
//
// 実方
class RGBRawReader : public Command
{
private:
    ColorSensor *colorSensor;
    rgb_raw_t rgbRaw;
    bool lockedRGBRawValue = false;

public:
    RGBRawReader(ColorSensor *colorSensor);
    void run();
    RGBRawReader *generateReverseCommand();
};

#endif