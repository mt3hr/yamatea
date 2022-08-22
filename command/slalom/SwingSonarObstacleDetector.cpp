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

    swingOrder = so;
    this->pwm = pwm;
    state = SSD_WAIT_START;
    stopper = new Stopper();
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
        rotateRobotDistanceAngleDetectorLeft = new RotateRobotDistanceAngleDetector(-swingLeft, targetLeft, pwm, robotAPI);
        rotateRobotDistanceAngleDetectorLeftPredicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetectorLeft);
        initedRotateRobotDistanceAngleDetectorLeft = true;
        return;
    }

    rotateRobotDistanceAngleDetectorLeft->run(robotAPI);

    if (rotateRobotDistanceAngleDetectorLeftPredicate->test(robotAPI))
    {
        leftObstacleDistance = rotateRobotDistanceAngleDetectorLeft->getDistance();
        leftObstacleAngle = rotateRobotDistanceAngleDetectorLeft->getAngle();
        writeDebug("SwingSonarObstacleDetector detectLeftObstacle finished");
        flushDebug(INFO, robotAPI);
        state = next;
    }
}

void SwingSonarObstacleDetector::returningLeft(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotCommandAndPreicateLeft)
    {
        initedRotateRobotCommandAndPreicateLeft = true;
        rotateRobotCommandAndPredicateLeft = new RotateRobotUseGyroCommandAndPredicate(ssodAbs(leftObstacleAngle), pwm, robotAPI);
        rotateRobotCommandAndPredicateLeft->getPredicate()->preparation(robotAPI);
    }

    rotateRobotCommandAndPredicateLeft->getCommand()->run(robotAPI);

    if (rotateRobotCommandAndPredicateLeft->getPredicate()->test(robotAPI))
    {
        writeDebug("SwingSonarObstacleDetector returningLeft finished");
        flushDebug(INFO, robotAPI);
        state = next;
    }
}

void SwingSonarObstacleDetector::detectRightObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotDistanceAngleDetectorRight)
    {
        initedRotateRobotDistanceAngleDetectorRight = true;
        rotateRobotDistanceAngleDetectorRight = new RotateRobotDistanceAngleDetector(swingRight, targetRight, pwm, robotAPI);
        rotateRobotDistanceAngleDetectorRightPredicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetectorRight);
    }

    rotateRobotDistanceAngleDetectorRight->run(robotAPI);

    if (rotateRobotDistanceAngleDetectorRightPredicate->test(robotAPI))
    {
        rightObstacleDistance = rotateRobotDistanceAngleDetectorRight->getDistance();
        rightObstacleAngle = rotateRobotDistanceAngleDetectorRight->getAngle();
        writeDebug("SwingSonarObstacleDetector detectRightObstacle finished");
        flushDebug(INFO, robotAPI);
        state = next;
    }
}

void SwingSonarObstacleDetector::returningRight(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotCommandAndPreicateRight)
    {
        initedRotateRobotCommandAndPreicateRight = true;
        rotateRobotCommandAndPredicateRight = new RotateRobotUseGyroCommandAndPredicate(-ssodAbs(rightObstacleAngle), pwm, robotAPI);
        rotateRobotCommandAndPredicateRight->getPredicate()->preparation(robotAPI);
    }

    rotateRobotCommandAndPredicateRight->getCommand()->run(robotAPI);

    if (rotateRobotCommandAndPredicateRight->getPredicate()->test(robotAPI))
    {
        writeDebug("SwingSonarObstacleDetector returningRight finished");
        flushDebug(INFO, robotAPI);
        state = next;
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
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_RETURNING_LEFT);
            if (state != SSD_RETURNING_LEFT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_RETURNING_LEFT:
        {
            returningLeft(robotAPI, SSD_DETECT_OBSTACLE_RIGHT);
            if (state != SSD_DETECT_OBSTACLE_RIGHT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_RETURNING_RIGHT);
            if (state != SSD_RETURNING_RIGHT)
            {
                break;
            }

            stopper->run(robotAPI);
        }

        case SSD_RETURNING_RIGHT:
        {
            returningRight(robotAPI, SSD_FINISHED);
            if (state != SSD_FINISHED)
            {
                break;
            }
            stopper->run(robotAPI);
            printValues(robotAPI);
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
    case CENTER_RIGHT_LEFT_CENTER:
    {
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_RIGHT;
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_RETURNING_RIGHT);
            if (state != SSD_RETURNING_RIGHT)
            {
                break;
            }

            stopper->run(robotAPI);
        }

        case SSD_RETURNING_RIGHT:
        {
            returningRight(robotAPI, SSD_DETECT_OBSTACLE_LEFT);
            if (state != SSD_DETECT_OBSTACLE_LEFT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_RETURNING_LEFT);
            if (state != SSD_RETURNING_LEFT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_RETURNING_LEFT:
        {
            returningLeft(robotAPI, SSD_FINISHED);
            if (state != SSD_FINISHED)
            {
                break;
            }
            stopper->run(robotAPI);
            printValues(robotAPI);
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

    case CENTER_LEFT_RIGHT:
    {
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_LEFT;
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_RETURNING_LEFT);
            if (state != SSD_RETURNING_LEFT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_RETURNING_LEFT:
        {
            returningLeft(robotAPI, SSD_DETECT_OBSTACLE_RIGHT);
            if (state != SSD_DETECT_OBSTACLE_RIGHT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_FINISHED);
            if (state != SSD_FINISHED)
            {
                break;
            }

            stopper->run(robotAPI);

            printValues(robotAPI);
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
        switch (state)
        {
        case SSD_WAIT_START:
        {
            state = SSD_DETECT_OBSTACLE_RIGHT;
        }

        case SSD_DETECT_OBSTACLE_RIGHT:
        {
            detectRightObstacle(robotAPI, SSD_RETURNING_RIGHT);
            if (state != SSD_RETURNING_RIGHT)
            {
                break;
            }

            stopper->run(robotAPI);
        }

        case SSD_RETURNING_RIGHT:
        {
            returningRight(robotAPI, SSD_DETECT_OBSTACLE_LEFT);
            if (state != SSD_DETECT_OBSTACLE_LEFT)
            {
                break;
            }
            stopper->run(robotAPI);
        }

        case SSD_DETECT_OBSTACLE_LEFT:
        {
            detectLeftObstacle(robotAPI, SSD_FINISHED);
            if (state != SSD_FINISHED)
            {
                break;
            }
            stopper->run(robotAPI);

            printValues(robotAPI);
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
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT_CENTER, pwm, swingLeft, swingRight, targetLeft, targetRight);
    case CENTER_RIGHT_LEFT_CENTER:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT_CENTER, pwm, swingLeft, swingRight, targetLeft, targetRight);
    case CENTER_LEFT_RIGHT:
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, pwm, swingLeft, swingRight, targetLeft, targetRight);
    case CENTER_RIGHT_LEFT:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, pwm, swingLeft, swingRight, targetLeft, targetRight);
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