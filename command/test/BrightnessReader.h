#ifndef BrighnessReader_H
#define BrightnessReader_H

#include "Command.h"
#include "RobotAPI.h"
#include "Sensor.h"

using namespace ev3api;

// 実方
class BrightnessReader : public Command
{
private:
    int brightness;
    bool lockedBrightnessValue = false;
    bool printedLockedBrightnessValue = false; // NOTE モデルには反映しません

public:
    BrightnessReader();
    virtual ~BrightnessReader();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual BrightnessReader *generateReverseCommand() override;
};

#endif