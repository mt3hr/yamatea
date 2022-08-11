#include "SwingSonarObstacleDetector.h"
#include "SonarSensor.h"
#include "RotateRobotDistanceAngleDetector.h"
#include "WheelController.h"
#include "FinishedCommandPredicate.h"
#include "RotateRobot.h"
#include "Stopper.h"
#include "string"         //TODO 消して
#include "sstream"        //TODO 消して
#include "vector"         //TODO 消して
#include "PrintMessage.h" //TODO 消して

using namespace ev3api;

SwingSonarObstacleDetector::SwingSonarObstacleDetector(SwingOrder so, int pwm, SonarSensor *ss, WheelController *wc)
{
    swingOrder = so;
    this->pwm = pwm;
    sonarSensor = ss;
    wheelController = wc;
    state = DETECT_OBSTACLE_1;
    stopper = new Stopper(wheelController);
};

SwingSonarObstacleDetector::~SwingSonarObstacleDetector()
{
    delete stopper;
    // TODO
};

void SwingSonarObstacleDetector::run()
{
    // TODO　コンストラクタ等引数に持っていって
    // 障害物の端っこを検出する形となるであろう
    float swingLeft = 45;   // 左方向への最大首振り角度
    float swingRight = -45; // 右方向への最大首振り角度

    int targetLeft = 20;  //左障害物判定のしきい値
    int targetRight = 20; //右障害物判定のしきい値

    switch (swingOrder)
    {
    case CENTER_LEFT_RIGHT:
    {
        switch (state)
        {
        case DETECT_OBSTACLE_1:
        {
            if (!initedRotateRobotDistanceAngleDetector1)
            {
                rotateRobotDistanceAngleDetector1 = new RotateRobotDistanceAngleDetector(swingLeft, targetLeft, pwm, wheelController, sonarSensor);
                rotateRobotDistanceAngleDetector1Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector1);
                initedRotateRobotDistanceAngleDetector1 = true;
                return;
            }

            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotDistanceAngleDetector1 == nullptr || rotateRobotDistanceAngleDetector1Predicate == nullptr)
            {
                return;
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
            stopper->run();
        }

        case RETURNING1:
        {
            if (!initedRotateRobotCommandAndPreicate1)
            {
                initedRotateRobotCommandAndPreicate1 = true;
                rotateRobotCommandAndPredicate1 = generateRotateRobotCommand(-(rotateRobotDistanceAngleDetector1->getAngle()), pwm, wheelController);
                rotateRobotCommandAndPredicate1->getPreHandler()->handle();
            }

            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotCommandAndPredicate1 == nullptr)
            {
                return;
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
            stopper->run();
        }

        case DETECT_OBSTACLE_2:
        {
            if (!initedRotateRobotDistanceAngleDetector2)
            {
                initedRotateRobotDistanceAngleDetector2 = true;
                rotateRobotDistanceAngleDetector2 = new RotateRobotDistanceAngleDetector(swingRight, targetRight, pwm, wheelController, sonarSensor);
                rotateRobotDistanceAngleDetector2Predicate = new FinishedCommandPredicate(rotateRobotDistanceAngleDetector2);
            }

            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotDistanceAngleDetector2 == nullptr || rotateRobotDistanceAngleDetector2Predicate == nullptr)
            {
                return;
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

            stopper->run();
        }

        case RETURNING2:
        {
            if (!initedRotateRobotCommandAndPreicate2)
            {
                initedRotateRobotCommandAndPreicate2 = true;
                rotateRobotCommandAndPredicate2 = generateRotateRobotCommand(rotateRobotDistanceAngleDetector2->getAngle(), pwm, wheelController);
                rotateRobotCommandAndPredicate2->getPreHandler()->handle();
            }
            // これがないとエラーが飛び得る。オブジェクトインスタンス化よりタスク呼び出し周期の方が速いことがあるらしい。
            if (rotateRobotCommandAndPredicate2 == nullptr)
            {
                return;
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
            stopper->run();
            leftObstacleDistance = rotateRobotDistanceAngleDetector1->getDistance();
            rightObstacleDistance = rotateRobotDistanceAngleDetector2->getDistance();
            obstacleAngle = rotateRobotDistanceAngleDetector1->getAngle() + rotateRobotDistanceAngleDetector2->getAngle();

            // 表示しちゃお
            stringstream d1s;                                                        // TODO 消して
            stringstream d2s;                                                        // TODO 消して
            stringstream as;                                                         // TODO 消して
            d1s.clear();                                                             // TODO 消して
            d2s.clear();                                                             // TODO 消して
            as.clear();                                                              // TODO 消して
            d1s.str("");                                                             // TODO 消して
            d2s.str("");                                                             // TODO 消して
            as.str("");                                                              // TODO 消して
            d1s << "distance1: " << float(getRightObstacleDistance());               // TODO 消して
            d2s << "distance2: " << float(getRightObstacleDistance());               // TODO 消して
            as << "angle: " << float(getObstacleAngle());                            // TODO 消して
            vector<string> messageLines;                                             // TODO 消して
            messageLines.push_back(d1s.str());                                       // TODO 消して
            messageLines.push_back(d2s.str());                                       // TODO 消して
            messageLines.push_back(as.str());                                        // TODO 消して
            PrintMessage *resultPrintCommand = new PrintMessage(messageLines, true); // TODO 消して
            resultPrintCommand->run();                                               // TODO 消して
            delete resultPrintCommand;                                               // TODO 消して
        }

        case FINISHED:
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
