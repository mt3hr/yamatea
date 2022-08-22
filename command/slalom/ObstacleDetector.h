#ifndef ObstacleDetector_H
#define ObstacleDetector_H

#include "Command.h"
#include "FinishConfirmable.h"

// ObstalceDetector
// 距離センサで2つの障害物間の距離を測るものの基底クラス
//
// 実方
class ObstacleDetector : public Command, public FinishConfirmable
{
private:
public:
    ~ObstacleDetector();
    virtual ObstacleDetector *generateReverseCommand() override;
    virtual int getLeftObstacleDistance();
    virtual int getRightObstacleDistance();
    virtual float getLeftObstacleAngle();
    virtual float getRightObstacleAngle();
    virtual bool isDetectedLeftObstacleDistance();
    virtual bool isDetectedRightObstacleDistance();
    virtual bool isDetectedLeftObstacleAngle();
    virtual bool isDetectedRightObstacleAngle();
};

#endif