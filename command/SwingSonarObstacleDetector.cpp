#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "WheelController.h"
#include "FinishedCommandPredicate.h"
#include "RotateRobot.h"
#include "Stopper.h"
#include "string"
#include "DebugUtil.h"

using namespace ev3api;

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so, int pwm, float swingLeft, float swingRight, int targetLeft, int targetRight, SonarSensor *ss, WheelController *wc)
{

    this->swingLeft = swingLeft;
    this->swingRight = swingRight;
    this->targetLeft = targetLeft;
    this->targetRight = targetRight;

    swingOrder = so;
    this->pwm = pwm;
    sonarSensor = ss;
    wheelController = wc;
    state = SSD_DETECT_OBSTACLE_1;
    stopper = new Stopper(wheelController);
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector()
{
    delete stopper;
    // TODO
};

void SwingSonarObstacleDetector::run()
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
                rotateRobotDistanceAngleDetector1 = new RotateRobotDistanceAngleDetector(swingLeft, targetLeft, pwm, wheelController, sonarSensor);
                rotateRobotDistanceAngleDetector1Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector1);
                initedRotateRobotDistanceAngleDetector1 = true;
                return;
            }

            /*
            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotDistanceAngleDetector1 == nullptr || rotateRobotDistanceAngleDetector1Predicate == nullptr)
            {
                return;
            }
            */

            rotateRobotDistanceAngleDetector1->run();

            if (rotateRobotDistanceAngleDetector1Predicate->test())
            {
                leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
                leftObstacleAngle = rotateRobotDistanceAngleDetector1->getAngle();
                state = SSD_RETURNING1;
            }

            if (state != SSD_RETURNING1)
            {
                break;
            }
            stopper->run();
        }

        case SSD_RETURNING1:
        {
            if (!initedRotateRobotCommandAndPreicate1)
            {
                initedRotateRobotCommandAndPreicate1 = true;
                rotateRobotCommandAndPredicate1 = generateRotateRobotCommand(-(rotateRobotDistanceAngleDetector1->getAngle()), pwm, wheelController);
                rotateRobotCommandAndPredicate1->getPreHandler()->handle();
            }

            /*
            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotCommandAndPredicate1 == nullptr)
            {
                return;
            }
            */

            rotateRobotCommandAndPredicate1->getCommand()->run();

            if (rotateRobotCommandAndPredicate1->getPredicate()->test())
            {
                state = SSD_DETECT_OBSTACLE_2;
            }

            if (state != SSD_DETECT_OBSTACLE_2)
            {
                break;
            }
            stopper->run();
        }

        case SSD_DETECT_OBSTACLE_2:
        {
            if (!initedRotateRobotDistanceAngleDetector2)
            {
                initedRotateRobotDistanceAngleDetector2 = true;
                rotateRobotDistanceAngleDetector2 = new RotateRobotDistanceAngleDetector(swingRight, targetRight, pwm, wheelController, sonarSensor);
                rotateRobotDistanceAngleDetector2Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector2);
            }

            /*
            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotDistanceAngleDetector2 == nullptr || rotateRobotDistanceAngleDetector2Predicate == nullptr)
            {
                return;
            }
            */

            rotateRobotDistanceAngleDetector2->run();

            if (rotateRobotDistanceAngleDetector2Predicate->test())
            {
                rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
                rightObstacleAngle = rotateRobotDistanceAngleDetector2->getAngle();
                state = SSD_RETURNING2;
            }

            if (state != SSD_RETURNING2)
            {
                break;
            }

            stopper->run();
        }

        case SSD_RETURNING2:
        {
            if (!initedRotateRobotCommandAndPreicate2)
            {
                initedRotateRobotCommandAndPreicate2 = true;
                rotateRobotCommandAndPredicate2 = generateRotateRobotCommand(rotateRobotDistanceAngleDetector2->getAngle(), pwm, wheelController);
                rotateRobotCommandAndPredicate2->getPreHandler()->handle();
            }

            /*
            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotCommandAndPredicate2 == nullptr)
            {
                return;
            }
            */

            rotateRobotCommandAndPredicate2->getCommand()->run();

            if (rotateRobotCommandAndPredicate2->getPredicate()->test())
            {
                state = SSD_FINISHED;
            }

            if (state != SSD_FINISHED)
            {
                break;
            }
            stopper->run();

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

        case SSD_FINISHED:
        {
            finished = true;
            stopper->run();
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
            return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, pwm, swingLeft, swingRight, targetLeft, targetRight, sonarSensor, wheelController);
        case CENTER_RIGHT_LEFT:
            return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, pwm, swingLeft, swingRight, targetLeft, targetRight, sonarSensor, wheelController);
        case LEFT_RIGHT:
            return new SwingSonarObstacleDetector(RIGHT_LEFT, pwm, swingLeft, swingRight, targetLeft, targetRight, sonarSensor, wheelController);
        case RIGHT_LEFT:
            return new SwingSonarObstacleDetector(LEFT_RIGHT, pwm, swingLeft, swingRight, targetLeft, targetRight, sonarSensor, wheelController);
            ;
        }
        return new SwingSonarObstacleDetector(swingOrder, pwm, swingLeft, swingRight, targetLeft, targetRight, sonarSensor, wheelController); // おかしい状態
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