#ifndef ColorIDReader_H
#define ColorIDReader_H

#include "Command.h"
#include "RobotAPI.h"
#include "Sensor.h"
#include "string"

using namespace ev3api;
using namespace std;

// ColorIDから色名を取得する関数
string colorIDToString(colorid_t colorID);

// ColorIDReader 
// ColorIDを測定するクラス。右ボタンで取得値記録（ディスプレイに表示）
// 
// 実方
class ColorIDReader : public Command
{
private:
    colorid_t colorID;
    bool lockedColorIDValue = false;
    bool printedLockedColorIDValue = false; // NOTE モデルには反映しません

public:
    ColorIDReader();
    virtual ~ColorIDReader();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorIDReader *generateReverseCommand() override;
};

#endif