#ifndef ObstacleDetector_H
#define ObstacleDetector_H

#include "Command.h"
#include "FinishConfirmable.h"

// SwingSonarDetector
// 距離センサで2つの障害物間の距離を測るクラス
//
// 実方
class ObstacleDetector : public Command, public FinishConfirmable
{
private:
public:
    ~ObstacleDetector();
    virtual int getLeftObstacleDistance();
    virtual int getRightObstacleDistance();
    virtual float getLeftObstacleAngle();
    virtual float getRightObstacleAngle();
    virtual bool isDetectedLeftObstacleDistance();
    virtual bool isDetectedRightObstacleDistance();
    virtual bool isDetectedLeftObstacleAngle();
    virtual bool isDetectedRightObstacleAngle();
    virtual ObstacleDetector *generateReverseCommand() override;
};

#endif