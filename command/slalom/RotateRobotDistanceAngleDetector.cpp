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
    flushDebug(DEBUG, robotAPI);

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
    rotateRobotCommand->run(robotAPI);
    float rawDistance = robotAPI->getSonarSensor()->getDistance();

#ifndef SimulatorMode
    float rawAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    float rawAngle = robotAPI->getGyroSensor()->getAngle();
#endif

    writeDebug("rawDistance: ");
    writeDebug(rawDistance);
    writeEndLineDebug();
    writeDebug("rawAngle: ");
    writeDebug(rawAngle);
    flushDebug(TRACE, robotAPI);

    if (rawDistance <= distanceThreshold)
    {
        distance = rawDistance;
        angle = rawAngle - angleWhenInited;
        detectedDistance = true;
        detectedAngle = true;
        writeDebug("distance: ");
        writeDebug(distance);
        writeEndLineDebug();
        writeDebug("angle: ");
        writeDebug(angle);
        flushDebug(TRACE, robotAPI);
    }

    if (isFinished())
    {
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
        return;
    }
}

void RotateRobotDistanceAngleDetector::preparation(RobotAPI *robotAPI)
{
    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
    rotateRobotPredicate->preparation(robotAPI);
#ifndef SimulatorMode
    angleWhenInited = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    angleWhenInited = robotAPI->getGyroSensor()->getAngle();
#endif

    writeDebug("RotateRobotDistanceAngleDetector.angleWhenInited: ");
    writeDebug(angleWhenInited);
    flushDebug(DEBUG, robotAPI);
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
    return detectedDistance;
}

bool RotateRobotDistanceAngleDetector::isDetectedAngle()
{
    return detectedAngle;
}
