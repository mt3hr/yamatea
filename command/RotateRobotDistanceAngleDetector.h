#ifndef RotateRobotDistanceAngleDetector_H
#define RotateRobotDistanceAngleDetector_H

#include "Command.h"
#include "FinishConfirmable.h"
#include "Predicate.h"
#include "MotorCountPredicate.h"
#include "Handler.h"
#include "SonarSensor.h"
#include "WheelController.h"

class RotateRobotDistanceAngleDetector : public Command, public FinishConfirmable
{
private:
    int pwm;
    float targetAngle;
    float angle;
    int distanceThreshold;
    int distance;

    SonarSensor *sonarSensor;
    WheelController *wheelController;

    Command *rotateRobotCommand;
    Predicate *rotateRobotPredicate;
    Handler *rotateRobotPreHandler;

    bool calledPreHandler = false;

    int leftWheelCountWhenInited = 0;
    int rightWheelCountWhenInited = 0;

public:
    RotateRobotDistanceAngleDetector(float angle, int distanceThreshold, int pwm, WheelController *wheelController, SonarSensor *sonarSensor);
    ~RotateRobotDistanceAngleDetector();
    void run() override;
    RotateRobotDistanceAngleDetector *generateReverseCommand() override;
    bool isFinished() override;
    int getDistance();
    float getAngle();
    bool isDetectedDistance();
    bool isDetectedAngle();
};

#endif