#ifndef ObstacleDetectRunner_H
#define ObstacleDetectRunner_H

#include "Command.h"

// ObstacleDetectRunner
// 難所スラロームペットボトル回避走行の基底クラス。
//
// 実方
class ObstacleDetectRunner : public Command
{
private:
    int leftObstacleDistance;
    int rightObstacleDistance;
public:
    ObstacleDetectRunner();
    int getLeftObstacleDistance();
    int getRightObstacleDistance();
};

#endif
