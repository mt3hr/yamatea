#include "RotateRobotDistanceAngleDetector.h"
#include "CommandAndPredicate.h"
#include "RotateRobot.h"
#include "WheelController.h"
#include "SonarSensor.h"
#include "Setting.h"
#include "Stopper.h"

RotateRobotDistanceAngleDetector::RotateRobotDistanceAngleDetector(float targetAngle, int distanceThreshold, int pwm, WheelController *wheelController, SonarSensor *sonarSensor)
{
    this->pwm = pwm;
    this->targetAngle = targetAngle;
    this->distanceThreshold = distanceThreshold;

    CommandAndPredicate *commandAndPredicate = generateRotateRobotCommand(targetAngle, pwm, wheelController);
    this->rotateRobotCommand = commandAndPredicate->getCommand();
    this->rotateRobotPredicate = commandAndPredicate->getPredicate();
    this->rotateRobotPreHandler = commandAndPredicate->getPreHandler();
    delete commandAndPredicate;

    this->wheelController = wheelController;
    this->sonarSensor = sonarSensor;
};

RotateRobotDistanceAngleDetector::~RotateRobotDistanceAngleDetector()
{
    delete rotateRobotCommand;
    delete rotateRobotPredicate;
    delete rotateRobotPreHandler;
}

void RotateRobotDistanceAngleDetector::run()
{
    if (!calledPreHandler)
    {
        calledPreHandler = true;
        rotateRobotPreHandler->handle();
        leftWheelCountWhenInited = wheelController->getLeftWheel()->getCount();
        rightWheelCountWhenInited = wheelController->getRightWheel()->getCount();
    }

    rotateRobotCommand->run();
    distance = sonarSensor->getDistance();

    // 360度回転するのに必要なモータ回転数:360 = totalWheelRotateAngle:x
    int totalWheelRotateAngle;

    // TODO ここあやしい
    if (targetAngle > 0)
    {
        totalWheelRotateAngle = (wheelController->getLeftWheel()->getCount() - leftWheelCountWhenInited);
        angle = float(360) * float(totalWheelRotateAngle) / float(angleFor360TurnLeftRotateRobot);
    }
    else
    {
        totalWheelRotateAngle = (wheelController->getRightWheel()->getCount() - rightWheelCountWhenInited);
        angle = (float(360) * float(totalWheelRotateAngle) / float(angleFor360TurnLeftRotateRobot));
    }

    if (isFinished())
    {
        Stopper *stopper = new Stopper(wheelController);
        stopper->run();
        delete stopper;
    }
}

RotateRobotDistanceAngleDetector *RotateRobotDistanceAngleDetector::generateReverseCommand()
{
    return new RotateRobotDistanceAngleDetector(-targetAngle, distanceThreshold, pwm, wheelController, sonarSensor);
}

bool RotateRobotDistanceAngleDetector::isFinished()
{
    return (isDetectedAngle() && isDetectedDistance()) || rotateRobotPredicate->test();
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
