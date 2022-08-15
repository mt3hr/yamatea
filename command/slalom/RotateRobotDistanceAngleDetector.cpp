#include "RotateRobotDistanceAngleDetector.h"
#include "CommandAndPredicate.h"
#include "RotateRobotCommandAndPredicate.h"
#include "SonarSensor.h"
#include "Setting.h"
#include "Stopper.h"

RotateRobotDistanceAngleDetector::RotateRobotDistanceAngleDetector(float targetAngle, int distanceThreshold, int pwm, RobotAPI *robotAPI)
{
    this->pwm = pwm;
    this->targetAngle = targetAngle;
    this->distanceThreshold = distanceThreshold;

    CommandAndPredicate *commandAndPredicate = new RotateRobotCommandAndPredicate(targetAngle, pwm, robotAPI);
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
        inited = true;
        rotateRobotPredicate->preparation(robotAPI);
        leftWheelCountWhenInited = robotAPI->getLeftWheel()->getCount();
        rightWheelCountWhenInited = robotAPI->getRightWheel()->getCount();
    }

    rotateRobotCommand->run(robotAPI);
    distance = robotAPI->getSonarSensor()->getDistance();

    // 360度回転するのに必要なモータ回転数:360 = totalWheelRotateAngle:x
    int totalWheelRotateAngle;

    if (targetAngle > 0)
    {
        totalWheelRotateAngle = (robotAPI->getLeftWheel()->getCount() - leftWheelCountWhenInited);
        angle = float(360) * float(totalWheelRotateAngle) / float(angleFor360TurnLeftRotateRobot);
    }
    else
    {
        totalWheelRotateAngle = (robotAPI->getRightWheel()->getCount() - rightWheelCountWhenInited);
        angle = (float(360) * float(totalWheelRotateAngle) / float(angleFor360TurnLeftRotateRobot));
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
