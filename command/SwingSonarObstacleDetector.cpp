#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "WheelController.h"
#include "FinishedCommandPredicate.h"
#include "RotateRobot.h"

using namespace ev3api;

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so, int pwm, SonarSensor *ss, WheelController *wc)
{
    state = DETECT_OBSTACLE_1;
    this->pwm = pwm;
    wheelController = wc;
    swingOrder = so;
    sonarSensor = ss;
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector(){
    // TODO
};

void SwingSonarObstacleDetector::run()
{
    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT:
    {
        // TODO　コンスト等引数に持っていって

        // 障害物の端っこを検出する形となるであろう
        float swingLeft = 45;   // 左方向への最大首振り角度
        float swingRight = -45; // 右方向への最大首振り角度

        int targetLeft = 10;  //左障害物判定のしきい値
        int targetRight = 10; //右障害物判定のしきい値

        switch (state)
        {
        case DETECT_OBSTACLE_1:
        {
            if (!initedRotateRobotDistanceAngleDetector1)
            {
                initedRotateRobotDistanceAngleDetector1 = true;
                rotateRobotDistanceAngleDetector1 = new RotateRobotDistanceAngleDetector(swingLeft, targetLeft, pwm, wheelController, sonarSensor);
                rotateRobotDistanceAngleDetector1Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector1);
            }

            rotateRobotDistanceAngleDetector1->run();

            if (rotateRobotDistanceAngleDetector1Predicate->test())
            {
                state = RETURNING1;
                leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            }

            if (state != RETURNING1)
            {
                break;
            }
        }

        case RETURNING1:
        {
            if (!initedRotateRobotCommandAndPreicate1)
            {
                initedRotateRobotCommandAndPreicate1 = true;
                rotateRobotCommandAndPredicate1 = generateRotateRobotCommand(-(rotateRobotDistanceAngleDetector1->getAngle()), pwm, wheelController);
                rotateRobotCommandAndPredicate1->getPreHandler()->handle();
            }
            rotateRobotCommandAndPredicate1->getCommand()->run();
            if (rotateRobotCommandAndPredicate1->getPredicate()->test())
            {
                state = DETECT_OBSTACLE_2;
            }

            if (state != DETECT_OBSTACLE_2)
            {
                break;
            }
        }

        case DETECT_OBSTACLE_2:
        {
            if (!initedRotateRobotDistanceAngleDetector2)
            {
                initedRotateRobotDistanceAngleDetector2 = true;
                rotateRobotDistanceAngleDetector2 = new RotateRobotDistanceAngleDetector(swingRight, targetRight, pwm, wheelController, sonarSensor);
                rotateRobotDistanceAngleDetector2Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector2);
            }

            rotateRobotDistanceAngleDetector2->run();

            if (rotateRobotDistanceAngleDetector2Predicate->test())
            {
                state = RETURNING2;
                rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
                obstacleAngle = rotateRobotDistanceAngleDetector1->getAngle() + rotateRobotDistanceAngleDetector2->getAngle();
            }

            if (state != RETURNING2)
            {
                break;
            }

            break;
        }

        case RETURNING2:
        {
            if (!initedRotateRobotCommandAndPreicate2)
            {
                initedRotateRobotCommandAndPreicate2 = true;
                rotateRobotCommandAndPredicate2 = generateRotateRobotCommand(rotateRobotDistanceAngleDetector2->getAngle(), pwm, wheelController);
                rotateRobotCommandAndPredicate2->getPreHandler()->handle();
            }
            rotateRobotCommandAndPredicate2->getCommand()->run();
            if (rotateRobotCommandAndPredicate2->getPredicate()->test())
            {
                state = FINISHED;
            }

            if (state != FINISHED)
            {
                break;
            }
            break;
        }

        case FINISHED:
        {
            finished = true;
            break;
        }

        default:
            break;
        }
        // TODO
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
        return new SwingSonarObstacleDetector(CENTER_RIGHT_LEFT, pwm, sonarSensor, wheelController);
    case CENTER_RIGHT_LEFT:
        return new SwingSonarObstacleDetector(CENTER_LEFT_RIGHT, pwm, sonarSensor, wheelController);
    case LEFT_RIGHT:
        return new SwingSonarObstacleDetector(RIGHT_LEFT, pwm, sonarSensor, wheelController);
    case RIGHT_LEFT:
        return new SwingSonarObstacleDetector(LEFT_RIGHT, pwm, sonarSensor, wheelController);
    }
    return new SwingSonarObstacleDetector(swingOrder, pwm, sonarSensor, wheelController); // おかしい状態
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

float SwingSonarObstacleDetector::getObstacleAngle()
{
    return obstacleAngle;
}

bool SwingSonarObstacleDetector::isDetectedLeftObstacleDistance()
{
    return detectedLeftObstacleDistance;
}

bool SwingSonarObstacleDetector::isDetectedRightObstacleDistance()
{
    return detectedRightObstacleDistance;
}

bool SwingSonarObstacleDetector::isDetectedObstacleAngle()
{
    return detectedObstacleAngle;
}
