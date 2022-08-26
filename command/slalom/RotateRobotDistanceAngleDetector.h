#ifndef RotateRobotDistanceAngleDetector_H
#define RotateRobotDistanceAngleDetector_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Predicate.h"
#include "MotorCountPredicate.h"
#include "RobotAPI.h"

// RotateRobotDistanceAngleDetector
// 障害物の距離と角度を旋回して測るコマンド。
// angleが+ならば時計回り、-ならば反時計回りをする。
// ソナーセンサでdistanceThreshold以下の値を検知したら停止する。（旋回前の方向に向き直ったりはしない）
// 
// 実方
class RotateRobotDistanceAngleDetector : public Command, public FinishConfirmable
{
private:
    int pwm;
    float targetAngle;
    float angle;
    int distanceThreshold;
    int distance;

    RobotAPI *robotAPI;
    Command *rotateRobotCommand;
    Predicate *rotateRobotPredicate;

    bool detectedDistance = false;
    bool detectedAngle = false;

    int angleWhenInited;

public:
    RotateRobotDistanceAngleDetector(float angle, int distanceThreshold, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotDistanceAngleDetector();
    virtual void run(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual RotateRobotDistanceAngleDetector *generateReverseCommand() override;
    virtual bool isFinished() override;
    virtual int getDistance();
    virtual float getAngle();
    virtual bool isDetectedDistance();
    virtual bool isDetectedAngle();
};

#endif