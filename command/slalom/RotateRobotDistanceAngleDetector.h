#ifndef RotateRobotDistanceAngleDetector_H
#define RotateRobotDistanceAngleDetector_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Predicate.h"
#include "MotorCountPredicate.h"
#include "RobotAPI.h"

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

    bool inited = false;
    bool detectedDistance = false;
    bool detectedAngle = false;

    int angleWhenInited;

public:
    RotateRobotDistanceAngleDetector(float angle, int distanceThreshold, int pwm, RobotAPI *robotAPI);
    virtual ~RotateRobotDistanceAngleDetector();
    virtual void run(RobotAPI *robotAPI) override;
    virtual RotateRobotDistanceAngleDetector *generateReverseCommand() override;
    virtual bool isFinished() override;
    virtual int getDistance();
    virtual float getAngle();
    virtual bool isDetectedDistance();
    virtual bool isDetectedAngle();
};

#endif