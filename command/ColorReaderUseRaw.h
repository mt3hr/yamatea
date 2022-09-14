#ifndef ColorReaderUseRaw_H
#define ColorReaderUseRaw_H

#include "Command.h"
#include "RobotAPI.h"
#include "RawColorPredicate.h"

using namespace ev3api;

class ColorReaderUseRaw : public Command
{
private:
    colorid_t *color;
    RedPredicate *redPredicate = new RedPredicate();
    GreenPredicate *greenPredicate = new GreenPredicate();
    BluePredicate *bluePredicate = new BluePredicate();
    YellowPredicate *yellowPredicate = new YellowPredicate();

public:
    ColorReaderUseRaw();
    virtual ~ColorReaderUseRaw();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual ColorReaderUseRaw *generateReverseCommand() override;
    virtual colorid_t *getColorPtr();
};
#endif