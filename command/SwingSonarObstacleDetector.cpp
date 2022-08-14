#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "FinishedCommandPredicate.h"
#include "RotateRobot.h"
#include "Stopper.h"
#include "string"
#include "DebugUtil.h"
#include "RobotAPI.h"

using namespace ev3api;

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so, int pwm, float swingLeft, float swingRight, int targetLeft, int targetRight)
{

    this->swingLeft = swingLeft;
    this->swingRight = swingRight;
    this->targetLeft = targetLeft;
    this->targetRight = targetRight;

    swingOrder = so;
    this->pwm = pwm;
    state = SSD_DETECT_OBSTACLE_1;
    stopper = new Stopper();
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector()
{
    delete stopper;
    // TODO
};

void SwingSonarObstacleDetector::run(RobotAPI *robotAPI)
{
    // 障害物の端っこを検出する形となるであろう
    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT:
    {
        switch (state)
        {
        case SSD_DETECT_OBSTACLE_1:
        {
            if (!initedRotateRobotDistanceAngleDetector1)
            {
                rotateRobotDistanceAngleDetector1 = new RotateRobotDistanceAngleDetector(swingLeft, targetLeft, pwm, robotAPI);
                rotateRobotDistanceAngleDetector1Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector1);
                initedRotateRobotDistanceAngleDetector1 = true;
                return;
            }

            rotateRobotDistanceAngleDetector1->run(robotAPI);

            if (rotateRobotDistanceAngleDetector1Predicate->test(robotAPI))
            {
                leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
                leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
                state = SSD_RETURNING1;
            }

            if (state != SSD_RETURNING1)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_RETURNING1:
        {
            if (!initedRotateRobotCommandAndPreicate1)
            {
                initedRotateRobotCommandAndPreicate1 = true;
                rotateRobotCommandAndPredicate1 = generateRotateRobotCommand(-(rotateRobotDistanceAngleDetector1->getAngle()), pwm, robotAPI);
                rotateRobotCommandAndPredicate1->getPredicate()->preparation(robotAPI);
            }

            rotateRobotCommandAndPredicate1->getCommand()->run(robotAPI);

            if (rotateRobotCommandAndPredicate1->getPredicate()->test(robotAPI))
            {
                state = SSD_DETECT_OBSTACLE_2;
            }

            if (state != SSD_DETECT_OBSTACLE_2)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_DETECT_OBSTACLE_2:
        {
            if (!initedRotateRobotDistanceAngleDetector2)
            {
                initedRotateRobotDistanceAngleDetector2 = true;
                rotateRobotDistanceAngleDetector2 = new RotateRobotDistanceAngleDetector(swingRight, targetRight, pwm, robotAPI);
                rotateRobotDistanceAngleDetector2Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector2);
            }

            rotateRobotDistanceAngleDetector2->run(robotAPI);

            if (rotateRobotDistanceAngleDetector2Predicate->test(robotAPI))
            {
                rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
                rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();
                state = SSD_RETURNING2;
            }

            if (state != SSD_RETURNING2)
            {
                break;
            }

            stopper->run(robotAPI);
        }

        case SSD_RETURNING2:
        {
            if (!initedRotateRobotCommandAndPreicate2)
            {
                initedRotateRobotCommandAndPreicate2 = true;
                rotateRobotCommandAndPredicate2 = generateRotateRobotCommand(rotateRobotDistanceAngleDetector2->getAngle(), pwm, robotAPI);
                rotateRobotCommandAndPredicate2->getPredicate()->preparation(robotAPI);
            }

            rotateRobotCommandAndPredicate2->getCommand()->run(robotAPI);

            if (rotateRobotCommandAndPredicate2->getPredicate()->test(robotAPI))
            {
                state = SSD_FINISHED;
            }

            if (state != SSD_FINISHED)
            {
                break;
            }
            stopper->run(robotAPI);

            leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
            leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
            rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();

            writeDebug("SwingSonarObstacleDetector");
            writeEndLineDebug();
            writeDebug("leftDistance: ");
            writeDebug(getLeftObstacleDistance());
            writeEndLineDebug();
            writeDebug("rightDistance: ");
            writeDebug(getRightObstacleAngle());
            writeEndLineDebug();
            writeDebug("leftAngle: ");
            writeDebug(getLeftObstacleAngle());
            writeEndLineDebug();
            writeDebug("rightAngle: ");
            writeDebug(getRightObstacleAngle());
            writeEndLineDebug();
            flushDebug(TRACE, robotAPI);
        }

        case SSD_FINISHED:
        {
            finished = true;
            stopper->run(robotAPI);
            break;
        }

        default:
            break;
        }
        break;
    }
    case CENTER_RIGHT_LEFT:
    {
        // TODO
        break;
    }
    case LEFT_RIGHT:
    {
        // TODO
        break;
    }
    case RIGHT_LEFT:
    {
        // TODO
        break;
    }

    default:
        break;
    }
}

SwingSonarObstacleDetector *SwingSonarObstacleDetector::generateReverseCommand()
{
    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT:
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, pwm, swingLeft, swingRight, targetLeft, targetRight);
    case CENTER_RIGHT_LEFT:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, pwm, swingLeft, swingRight, targetLeft, targetRight);
    case LEFT_RIGHT:
        return new SwingSonarObstacleDetector(RIGHT_LEFT, pwm, swingLeft, swingRight, targetLeft, targetRight);
    case RIGHT_LEFT:
        return new SwingSonarObstacleDetector(LEFT_RIGHT, pwm, swingLeft, swingRight, targetLeft, targetRight);
    }
    return new SwingSonarObstacleDetector(swingOrder, pwm, swingLeft, swingRight, targetLeft, targetRight); // おかしい状態
}

bool SwingSonarObstacleDetector::isFinished()
{
    return finished;
}

int SwingSonarObstacleDetector::getLeftObstacleDistance()
{
    return leftObstacleDistance;
}

int SwingSonarObstacleDetector::getRightObstacleDistance()
{
    return rightObstacleDistance;
}

float SwingSonarObstacleDetector::getLeftObstacleAngle()
{
    return leftObstacleAngle;
}

float SwingSonarObstacleDetector::getRightObstacleAngle()
{
    return rightObstacleAngle;
}

bool SwingSonarObstacleDetector::isDetectedLeftObstacleDistance()
{
    return detectedLeftObstacleDistance;
}

bool SwingSonarObstacleDetector::isDetectedRightObstacleDistance()
{
    return detectedRightObstacleDistance;
}

bool SwingSonarObstacleDetector::isDetectedLeftObstacleAngle()
{
    return detectedLeftObstacleAngle;
}

bool SwingSonarObstacleDetector::isDetectedRightObstacleAngle()
{
    return detectedRightObstacleAngle;
}