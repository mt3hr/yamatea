#include "RotateRobotDistanceAngleDetector.h"
#include "CommandAndPredicate.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "SonarSensor.h"
#include "Setting.h"
#include "Stopper.h"
#include "DebugUtil.h"

RotateRobotDistanceAngleDetector::RotateRobotDistanceAngleDetector(float targetAngle, int distanceThreshold, int pwm, RobotAPI *robotAPI)
{
    this->robotAPI = robotAPI;
    this->pwm = pwm;
    this->targetAngle = targetAngle;
    this->distanceThreshold = distanceThreshold;

    writeDebug("RotateRobotDistanceAngleDetector.targetAngle: ");
    writeDebug(targetAngle);
    flushDebug(TRACE, robotAPI);

    CommandAndPredicate *commandAndPredicate = new RotateRobotUseGyroCommandAndPredicate(targetAngle, pwm, robotAPI);
    this->rotateRobotCommand = commandAndPredicate->getCommand();
    this->rotateRobotPredicate = commandAndPredicate->getPredicate();
    delete commandAndPredicate;
};

RotateRobotDistanceAngleDetector::~RotateRobotDistanceAngleDetector()
{
    delete rotateRobotCommand;
    delete rotateRobotPredicate;
}

void RotateRobotDistanceAngleDetector::run(RobotAPI *robotAPI)
{
    if (!inited)
    {
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
        rotateRobotPredicate->preparation(robotAPI);
        angleWhenInited = robotAPI->getGyroSensor()->getAngle();

#ifndef SimulatorMode
        angleWhenInited *= -1;
#endif

        writeDebug("RotateRobotDistanceAngleDetector.angleWhenInited: ");
        writeDebug(angleWhenInited);
        flushDebug(DEBUG, robotAPI);
        inited = true;
    }

    rotateRobotCommand->run(robotAPI);
    distance = robotAPI->getSonarSensor()->getDistance();
    distance = robotAPI->getSonarSensor()->getDistance(); // 2回読み取ってみるか
    float rawAngle = robotAPI->getGyroSensor()->getAngle();

#ifndef SimulatorMode
    rawAngle *= -1;
#endif

    angle = rawAngle - angleWhenInited; // 分度器で角度をはかる都合で時計回りを+にしたいｔめ、-1をかける

    writeDebug("distance: ");
    writeDebug(distance);
    flushDebug(TRACE, robotAPI);
    writeDebug("angle: ");
    writeDebug(angle);
    flushDebug(TRACE, robotAPI);

    if (isFinished())
    {
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
        return;
    }
}

RotateRobotDistanceAngleDetector *RotateRobotDistanceAngleDetector::generateReverseCommand()
{
    return new RotateRobotDistanceAngleDetector(-targetAngle, distanceThreshold, pwm, robotAPI);
}

bool RotateRobotDistanceAngleDetector::isFinished()
{
    return (isDetectedAngle() && isDetectedDistance()) || rotateRobotPredicate->test(robotAPI);
}

int RotateRobotDistanceAngleDetector::getDistance()
{
    return distance;
}

float RotateRobotDistanceAngleDetector::getAngle()
{
    return angle;
}

bool RotateRobotDistanceAngleDetector::isDetectedDistance()
{
    return getDistance() <= distanceThreshold;
}

bool RotateRobotDistanceAngleDetector::isDetectedAngle()
{
    return getDistance() <= distanceThreshold;
}
