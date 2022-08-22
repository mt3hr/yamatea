#ifndef ClockwiseObstacleDetector_H
#define ClockwiseObstacleDetector_H

#include "ObstacleDetector.h"
#include "Walker.h"
#include "Stopper.h"

enum ClockwiseObstacleDetectorState
{
    CODS_DETECTING_LEFT_OBSTACLE,
    CODS_DETECTING_RIGHT_OBSTACLE,
    CODS_FINISH,
};

// ClockwiseObstacleDetector
// 2点の障害物間の距離と角度を時計回りに旋回しながら測定するDetector。
// runが呼び出される時点で、すでに左のオブジェクトを検出している必要がある
//
// 実方
class ClockwiseObstacleDetector : public ObstacleDetector
{
private:
    ClockwiseObstacleDetectorState state;
    int pwm;
    float angle;
    int thresholdDistance;
    int targetLeft;
    int targetRight;

    bool reverse = false;

    Walker *turnWalker;
    Stopper *stopper;

    int leftObstacleDistance = 256;
    int rightObstacleDistance = 256;
    float leftObstacleAngle = 0;
    float rightObstacleAngle = 0;

    bool detectedLeftObstacleDistance = false;
    bool detectedRightObstacleDistance = false;
    bool detectedLeftObstacleAngle = false;
    bool detectedRightObstacleAngle = false;

    float targetAngle;

public:
    ClockwiseObstacleDetector(int pwm, float angle, int thresholdDistance, int targetLeft, int targetRight);
    virtual ~ClockwiseObstacleDetector();
    virtual void run(RobotAPI *robotAPI) override;
    virtual ClockwiseObstacleDetector *generateReverseCommand() override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual bool isFinished() override;
    virtual int getLeftObstacleDistance() override;
    virtual int getRightObstacleDistance() override;
    virtual float getLeftObstacleAngle() override;
    virtual float getRightObstacleAngle() override;
    virtual bool isDetectedLeftObstacleDistance() override;
    virtual bool isDetectedRightObstacleDistance() override;
    virtual bool isDetectedLeftObstacleAngle() override;
    virtual bool isDetectedRightObstacleAngle() override;
};

#endif