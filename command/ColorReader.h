#ifndef ColorReader_H
#define ColorReader_H

#include "Command.h"
#include "RobotAPI.h"
#include "Sensor.h"

class ColorReader : public Command
{
private:
    colorid_t color;
public:
    ColorReader();
    virtual ~ColorReader();
    virtual void run(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual Command *generateReverseCommand();
    virtual colorid_t getColor();
};

#endif
