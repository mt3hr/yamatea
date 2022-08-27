#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "FinishedCommandPredicate.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "Stopper.h"
#include "string"
#include "DebugUtil.h"
#include "RobotAPI.h"
#include "Setting.h"

using namespace ev3api;

float ssodAbs(float f)
{
    if (f < 0)
    {
        return -f;
    }
    return f;
}

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so, int pwm, float swingLeft, float swingRight, int targetLeft, int targetRight)
{
    this->swingLeft = swingLeft;
    this->swingRight = swingRight;
    this->targetLeft = targetLeft;
    this->targetRight = targetRight;

    this->swingOrder = so;
    this->pwm = pwm;
    this->stopper = new Stopper();
    this->state = SSD_WAIT_START;
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector()
{
    delete stopper;
    delete rotateRobotDistanceAngleDetectorLeft;
    delete rotateRobotDistanceAngleDetectorRight;
    delete rotateRobotDistanceAngleDetectorLeftPredicate;
    delete rotateRobotDistanceAngleDetectorRightPredicate;
    delete rotateRobotCommandAndPredicateLeft;
    delete rotateRobotCommandAndPredicateRight;
};

void SwingSonarObstacleDetector::detectLeftObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotDistanceAngleDetectorLeft)
    {
        rotateRobotDistanceAngleDetectorLeft = new RotateRobotDistanceAngleDetector(swingLeft, targetLeft, pwm, robotAPI);
        rotateRobotDistanceAngleDetectorLeftPredicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetectorLeft);
        rotateRobotDistanceAngleDetectorLeft->preparation(robotAPI);
        rotateRobotDistanceAngleDetectorLeftPredicate->preparation(robotAPI);
        initedRotateRobotDistanceAngleDetectorLeft = true;
        return;
    }

    rotateRobotDistanceAngleDetectorLeft->run(robotAPI);

    if (rotateRobotDistanceAngleDetectorLeftPredicate->test(robotAPI))
    {
        leftObstacleDistance = rotateRobotDistanceAngleDetectorLeft->getDistance();
        leftObstacleAngle = rotateRobotDistanceAngleDetectorLeft->getAngle();
        state = next;
        writeDebug("SwingSonarObstacleDetector detectLeftObstacle finished");
        flushDebug(INFO, robotAPI);
    }
}

void SwingSonarObstacleDetector::returningLeft(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotCommandAndPreicateLeft)
    {
        initedRotateRobotCommandAndPreicateLeft = true;
        rotateRobotCommandAndPredicateLeft = new RotateRobotUseGyroCommandAndPredicate(-leftObstacleAngle, pwm, robotAPI);
        rotateRobotCommandAndPredicateLeft->getCommand()->preparation(robotAPI);
        rotateRobotCommandAndPredicateLeft->getPredicate()->preparation(robotAPI);
    }

    rotateRobotCommandAndPredicateLeft->getCommand()->run(robotAPI);

    if (rotateRobotCommandAndPredicateLeft->getPredicate()->test(robotAPI))
    {
        state = next;
        writeDebug("SwingSonarObstacleDetector returningLeft finished");
        flushDebug(INFO, robotAPI);
    }
}

void SwingSonarObstacleDetector::detectRightObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotDistanceAngleDetectorRight)
    {
        rotateRobotDistanceAngleDetectorRight = new RotateRobotDistanceAngleDetector(swingRight, targetRight, pwm, robotAPI);
        rotateRobotDistanceAngleDetectorRightPredicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetectorRight);
        rotateRobotDistanceAngleDetectorRight->preparation(robotAPI);
        rotateRobotDistanceAngleDetectorRightPredicate->preparation(robotAPI);
        initedRotateRobotDistanceAngleDetectorRight = true;
        return;
    }

    rotateRobotDistanceAngleDetectorRight->run(robotAPI);

    if (rotateRobotDistanceAngleDetectorRightPredicate->test(robotAPI))
    {
        rightObstacleDistance = rotateRobotDistanceAngleDetectorRight->getDistance();
        rightObstacleAngle = rotateRobotDistanceAngleDetectorRight->getAngle();
        state = next;
        writeDebug("SwingSonarObstacleDetector detectRightObstacle finished");
        flushDebug(INFO, robotAPI);
    }
}

void SwingSonarObstacleDetector::returningRight(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotCommandAndPreicateRight)
    {
        initedRotateRobotCommandAndPreicateRight = true;
        rotateRobotCommandAndPredicateRight = new RotateRobotUseGyroCommandAndPredicate(-rightObstacleAngle, pwm, robotAPI);
        rotateRobotCommandAndPredicateRight->getCommand()->preparation(robotAPI);
        rotateRobotCommandAndPredicateRight->getPredicate()->preparation(robotAPI);
    }

    rotateRobotCommandAndPredicateRight->getCommand()->run(robotAPI);

    if (rotateRobotCommandAndPredicateRight->getPredicate()->test(robotAPI))
    {
        state = next;
        writeDebug("SwingSonarObstacleDetector returningRight finished");
        flushDebug(INFO, robotAPI);
    }
}

void SwingSonarObstacleDetector::printValues(RobotAPI *robotAPI)
{
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

void SwingSonarObstacleDetector::run(RobotAPI *robotAPI)
{
    // 障害物の端っこを検出する形となるであろう
    switch (swingOrder)
    {

    case CENTER_LEFT_RIGHT_CENTER:
    {
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_LEFT;
            break;
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_RETURNING_LEFT);
            break;
        }

        case SSD_RETURNING_LEFT:
        {
            returningLeft(robotAPI, SSD_DETECT_OBSTACLE_RIGHT);
            break;
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_RETURNING_RIGHT);
            break;
        }

        case SSD_RETURNING_RIGHT:
        {
            returningRight(robotAPI, SSD_FINISHED);
            break;
        }

        case SSD_FINISHED:
        {
            finished = true;
            stopper->run(robotAPI);
            printValues(robotAPI);
            break;
        }

        default:
            break;
        }
        break;
    }
    case CENTER_RIGHT_LEFT_CENTER:
    {
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_RIGHT;
            break;
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_RETURNING_RIGHT);
            break;
        }

        case SSD_RETURNING_RIGHT:
        {
            returningRight(robotAPI, SSD_DETECT_OBSTACLE_LEFT);
            break;
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_RETURNING_LEFT);
            break;
        }

        case SSD_RETURNING_LEFT:
        {
            returningLeft(robotAPI, SSD_FINISHED);
            break;
        }

        case SSD_FINISHED:
        {
            finished = true;
            stopper->run(robotAPI);
            printValues(robotAPI);
            break;
        }

        default:
            break;
        }
        break;
    }

    case CENTER_LEFT_RIGHT:
    {
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_LEFT;
            break;
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_RETURNING_LEFT);
            break;
        }

        case SSD_RETURNING_LEFT:
        {
            returningLeft(robotAPI, SSD_DETECT_OBSTACLE_RIGHT);
            break;
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_FINISHED);
            break;
        }

        case SSD_FINISHED:
        {
            finished = true;
            stopper->run(robotAPI);
            printValues(robotAPI);
            break;
        }

        default:
            break;
        }
        break;
    }
    case CENTER_RIGHT_LEFT:
    {
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_RIGHT;
            break;
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_RETURNING_RIGHT);
            break;
        }

        case SSD_RETURNING_RIGHT:
        {
            returningRight(robotAPI, SSD_DETECT_OBSTACLE_LEFT);
            break;
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_FINISHED);
            break;
        }

        case SSD_FINISHED:
        {
            finished = true;
            stopper->run(robotAPI);
            printValues(robotAPI);
            break;
        }

        default:
            break;
        }
        break;
    }

    default:
        break;
    }
}

void SwingSonarObstacleDetector::preparation(RobotAPI *robotAPI)
{
    return;
}

SwingSonarObstacleDetector *SwingSonarObstacleDetector::generateReverseCommand()
{
    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT_CENTER:
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT_CENTER, pwm, swingRight, swingLeft, targetRight, targetLeft);
    case CENTER_RIGHT_LEFT_CENTER:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT_CENTER, pwm, swingRight, swingLeft, targetRight, targetLeft);
    case CENTER_LEFT_RIGHT:
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, pwm, swingRight, swingLeft, targetRight, targetLeft);
    case CENTER_RIGHT_LEFT:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, pwm, swingRight, swingLeft, targetRight, targetLeft);
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