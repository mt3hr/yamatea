#ifndef CheetBlock_H
#define CheetBlock_H


#include "ColorReader.h"
#include "RobotAPI.h"
#include "CommandExecutor.h"

enum CheetBlockState
{
    CR_R,
    CR_B,
    CR_G,
    CR_Y
};

// 小路
class DealingWithGarage : public Command
{
private:
    bool executeState = false;
    bool reverse;
    colorid_t *colorID;
    CommandExecutor* commandExecutor;

public:
    DealingWithGarage(colorid_t *colorID,CommandExecutor* commandExecutor,bool reverse);
    virtual ~DealingWithGarage() override;
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Command *generateReverseCommand();
};

#endif