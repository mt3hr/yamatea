#ifndef DistanceReader_H
#define DistanceReader_H

#include "SonarSensor.h"
#include "Command.h"
#include "RobotAPI.h"

using namespace ev3api;

// DistanceReader
// 距離を測定するクラス。
// 試走会などで値を取るためのもの
//
// 実方
class DistanceReader : public Command
{
private:
    int distanceValue = 0;

public:
    DistanceReader();
    void run(RobotAPI *robotAPI);
    DistanceReader *generateReverseCommand();
};

#endif