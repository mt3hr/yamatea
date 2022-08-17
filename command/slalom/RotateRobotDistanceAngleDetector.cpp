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
        rotateRobotPredicate->preparation(robotAPI);
        angleWhenInited = robotAPI->getGyroSensor()->getAngle() * -1;
        writeDebug("RotateRobotDistanceAngleDetector.angleWhenInited: ");
        writeDebug(angleWhenInited);
        flushDebug(DEBUG, robotAPI);
        inited = true;
    }

    rotateRobotCommand->run(robotAPI);
    distance = robotAPI->getSonarSensor()->getDistance();

    angle = (robotAPI->getGyroSensor()->getAngle() * -1) + angleWhenInited; // 分度器で角度をはかる都合で時計回りを+にしたいｔめ、-1をかける
    // 角度から符号を消す
    if (angle < 0)
    {
        angle *= -1;
    }

    if (isFinished())
    {
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
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
