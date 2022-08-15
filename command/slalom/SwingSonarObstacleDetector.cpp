#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "FinishedCommandPredicate.h"
#include "RotateRobotCommandAndPredicate.h"
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
    state = SSD_WAIT_START;
    stopper = new Stopper();
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector()
{
    delete stopper;
    // TODO
};

void SwingSonarObstacleDetector::detectLeftObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
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
        state = next;
    }
}

void SwingSonarObstacleDetector::returningLeft(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotCommandAndPreicate1)
    {
        initedRotateRobotCommandAndPreicate1 = true;
        rotateRobotCommandAndPredicate1 = new RotateRobotCommandAndPredicate(-(rotateRobotDistanceAngleDetector1->getAngle()), pwm, robotAPI);
        rotateRobotCommandAndPredicate1->getPredicate()->preparation(robotAPI);
    }

    rotateRobotCommandAndPredicate1->getCommand()->run(robotAPI);

    if (rotateRobotCommandAndPredicate1->getPredicate()->test(robotAPI))
    {
        state = next;
    }
}

void SwingSonarObstacleDetector::detectRightObstacle(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
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
        state = next;
    }
}

void SwingSonarObstacleDetector::returningRight(RobotAPI *robotAPI, SwingSonarObstacleDetectorState next)
{
    if (!initedRotateRobotCommandAndPreicate2)
    {
        initedRotateRobotCommandAndPreicate2 = true;
        rotateRobotCommandAndPredicate2 = new RotateRobotCommandAndPredicate(rotateRobotDistanceAngleDetector2->getAngle(), pwm, robotAPI);
        rotateRobotCommandAndPredicate2->getPredicate()->preparation(robotAPI);
    }

    rotateRobotCommandAndPredicate2->getCommand()->run(robotAPI);

    if (rotateRobotCommandAndPredicate2->getPredicate()->test(robotAPI))
    {
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

            leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
            leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
            rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();

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

            leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
            leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
            rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();

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

            leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
            leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
            rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();

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

            leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
            leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
            rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();

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