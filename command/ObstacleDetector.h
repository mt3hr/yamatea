#ifndef ObstacleDetector_H
#define ObstacleDetector_H

#include "Command.h"

// SwingSonarDetector
// 距離センサで2つの障害物間の距離を測るクラス
//
// 実方
class ObstacleDetector : public Command
{
private:
public:
    ~ObstacleDetector();
    virtual int getLeftObstacleDistance();
    virtual int getRightObstacleDistance();
    virtual float getObstacleAngle();
    virtual bool isDetectedLeftObstacleDistance();
    virtual bool isDetectedRightObstacleDistance();
    virtual bool isDetectedObstacleAngle();
    virtual ObstacleDetector *generateReverseCommand() override;
};

#endif