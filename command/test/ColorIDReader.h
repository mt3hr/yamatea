#ifndef ColorIDReader_H
#define ColorIDReader_H

#include "Command.h"
#include "RobotAPI.h"
#include "Sensor.h"

using namespace ev3api;

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