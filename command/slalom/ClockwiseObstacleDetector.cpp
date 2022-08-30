#include "ClockwiseObstacleDetector.h"
#include "Stopper.h"
#include "DebugUtil.h"

// 距離センサから取得できる値がthresholdDistance以上になったタイミングで、最初のオブジェクトの距離を確定します。
ClockwiseObstacleDetector::ClockwiseObstacleDetector(int pwm, float angle, int targetLeft, int targetRight, int skipFrameAfterDetectFirstObstacle, bool facingObstacle)
{
    this->state = CODS_DETECTING_LEFT_OBSTACLE;
    this->turnWalker = new Walker(pwm, -pwm);
    this->stopper = new Stopper();
    this->pwm = pwm;
    this->angle = angle;
    this->targetLeft = targetLeft;
    this->targetRight = targetRight;
    this->skipFrameAfterDetectFirstObstacle = skipFrameAfterDetectFirstObstacle;
    this->reverse = false;
    this->ignoreFrameWhenFirstDetected = new NumberOfTimesPredicate(skipFrameAfterDetectFirstObstacle);       // TODO モデルとコメント、引数化
    this->ignoreFrameWhenFirstBeforeDetected = new NumberOfTimesPredicate(0); // TODO モデルとコメント、引数化
    this->facingObstacle = facingObstacle;
    this->detectedFirstObstacle = !facingObstacle;
};

ClockwiseObstacleDetector::~ClockwiseObstacleDetector()
{
    delete turnWalker;
    delete stopper;
};

void ClockwiseObstacleDetector::measure(RobotAPI *robotAPI)
{
    preDistance = currentDistance;
    preAngle = currentAngle;
    currentDistance = robotAPI->getSonarSensor()->getDistance();
#ifndef SimulatorMode
    currentAngle = robotAPI->getGyroSensor()->getAngle() * -1; // TODO - angleOffset;
#else
    currentAngle = robotAPI->getGyroSensor()->getAngle(); // TODO - angleOffset;
#endif

    writeDebug("currentDistance: ");
    writeDebug(currentDistance);
    writeEndLineDebug();
    writeDebug("currentAngle: ");
    writeDebug(currentAngle);
    flushDebug(TRACE, robotAPI);
}

void ClockwiseObstacleDetector::run(RobotAPI *robotAPI)
{
    measure(robotAPI);
    if (!reverse)
    {
        if (targetAngle <= currentAngle)
        {
            stopper->run(robotAPI);
            state = CODS_FINISH;

            writeDebug("finishClockwiseObstacleDetector");
            flushDebug(DEBUG, robotAPI);
            return;
        }
        switch (state)
        {
        case CODS_DETECTING_LEFT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            switch (detectedFirstObstacle)
            {
            case false:
            {
                if (targetLeft >= currentDistance)
                {
                    detectedFirstObstacle = true;
                }
                else
                {
                    return;
                }
            }

            case true:
            {
                if (!facingObstacle)
                {
                    if (!ignoreFrameWhenFirstBeforeDetected->test(robotAPI))
                    {
                        return;
                    }
                }
                if (targetLeft < currentDistance)
                {
                    leftObstacleDistance = preDistance;
                    leftObstacleAngle = preAngle;
                    detectedLeftObstacleDistance = true;
                    detectedLeftObstacleAngle = true;
                    state = CODS_DETECTING_RIGHT_OBSTACLE;
                }
            }
            }
            if (state == CODS_DETECTING_RIGHT_OBSTACLE)
            {
                break;
            }

            writeDebug("CODS_DETECTING_LEFT_OBSTACLE");
            flushDebug(DEBUG, robotAPI);
        }
        case CODS_DETECTING_RIGHT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            if (!ignoreFrameWhenFirstDetected->test(robotAPI))
            {
                return;
            }

            if (targetLeft >= currentDistance)
            {
                rightObstacleDistance = currentDistance;
                rightObstacleAngle = currentAngle;
                detectedRightObstacleDistance = true;
                detectedRightObstacleAngle = true;
                state = CODS_FINISH;
            }
            if (state != CODS_FINISH)
            {
                return;
            }
            stopper->run(robotAPI);

            writeDebug("CODS_DETECTING_RIGHT_OBSTACLE");
            flushDebug(DEBUG, robotAPI);
            printValues(robotAPI);
        }
        case CODS_FINISH:
        {
            stopper->run(robotAPI);
            return;
        }
        default:
            return;
        }
    }
    else
    {
        if (currentAngle >= targetAngle)
        {
            state = CODS_FINISH;
            stopper->run(robotAPI);

            writeDebug("ClockwiseObstacleDetector finished");
            flushDebug(DEBUG, robotAPI);
            return;
        }
        switch (state)
        {
        case CODS_DETECTING_RIGHT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            switch (detectedFirstObstacle)
            {
            case false:
            {
                if (targetRight >= currentDistance)
                {
                    detectedFirstObstacle = true;
                }
                else
                {
                    return;
                }
            }

            case true:
            {
                if (!facingObstacle)
                {
                    if (!ignoreFrameWhenFirstBeforeDetected->test(robotAPI))
                    {
                        return;
                    }
                }
                if (targetRight < currentDistance)
                {
                    rightObstacleDistance = preDistance;
                    rightObstacleAngle = preAngle;
                    detectedRightObstacleDistance = true;
                    detectedRightObstacleAngle = true;
                    state = CODS_DETECTING_LEFT_OBSTACLE;
                }
            }
            }
            if (state != CODS_DETECTING_LEFT_OBSTACLE)
            {
                return;
            }

            writeDebug("CODS_DETECTING_RIGHT_OBSTACLE");
            flushDebug(DEBUG, robotAPI);
        }
        case CODS_DETECTING_LEFT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            if (!ignoreFrameWhenFirstDetected->test(robotAPI))
            {
                return;
            }

            if (targetRight >= currentDistance)
            {
                leftObstacleDistance = currentDistance;
                leftObstacleAngle = currentAngle;
                detectedLeftObstacleDistance = true;
                detectedLeftObstacleAngle = true;
                state = CODS_FINISH;
            }
            if (state != CODS_FINISH)
            {
                return;
            }
            stopper->run(robotAPI);

            writeDebug("CODS_DETECTING_LEFT_OBSTACLE");
            flushDebug(DEBUG, robotAPI);
            printValues(robotAPI);
        }
        case CODS_FINISH:
            stopper->run(robotAPI);
        default:
            return;
        }
        return;
    }
}

void ClockwiseObstacleDetector::printValues(RobotAPI *robotAPI)
{
    writeDebug("leftAngle:");
    writeDebug(isDetectedLeftObstacleAngle());
    writeEndLineDebug();
    writeDebug("leftDistance:");
    writeDebug(isDetectedLeftObstacleDistance());
    writeEndLineDebug();
    writeDebug("rightAngle: ");
    writeDebug(isDetectedRightObstacleAngle());
    writeEndLineDebug();
    writeDebug("rightDistance: ");
    writeDebug(isDetectedRightObstacleDistance());
    flushDebug(DEBUG, robotAPI);
}

ClockwiseObstacleDetector *ClockwiseObstacleDetector::generateReverseCommand()
{
    ClockwiseObstacleDetector *reversed = new ClockwiseObstacleDetector(-pwm, -angle, targetRight, targetLeft, skipFrameAfterDetectFirstObstacle, facingObstacle);
    reversed->reverse = !reverse;
    if (reversed->reverse)
    {
        reversed->state = CODS_DETECTING_RIGHT_OBSTACLE;
    }
    else
    {
        reversed->state = CODS_DETECTING_LEFT_OBSTACLE;
    }
    return reversed;
}

void ClockwiseObstacleDetector::preparation(RobotAPI *robotAPI)
{
#ifndef SimulatorMode
    float currentAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
    float currentAngle = robotAPI->getGyroSensor()->getAngle();
#endif
    angleOffset = currentAngle;
    if (angle > 0)
    {
        targetAngle = currentAngle + angle;
    }
    else
    {
        targetAngle = currentAngle - angle;
    }
}

bool ClockwiseObstacleDetector::isFinished()
{
    return state == CODS_FINISH;
}

int ClockwiseObstacleDetector::getLeftObstacleDistance()
{
    return leftObstacleDistance;
}

int ClockwiseObstacleDetector::getRightObstacleDistance()
{
    return rightObstacleDistance;
}

float ClockwiseObstacleDetector::getLeftObstacleAngle()
{
    return leftObstacleAngle;
}

float ClockwiseObstacleDetector::getRightObstacleAngle()
{
    return rightObstacleAngle;
}

bool ClockwiseObstacleDetector::isDetectedLeftObstacleDistance()
{
    return detectedLeftObstacleDistance;
}

bool ClockwiseObstacleDetector::isDetectedRightObstacleDistance()
{
    return detectedRightObstacleDistance;
}

bool ClockwiseObstacleDetector::isDetectedLeftObstacleAngle()
{
    return detectedLeftObstacleAngle;
}

bool ClockwiseObstacleDetector::isDetectedRightObstacleAngle()
{
    return detectedRightObstacleAngle;
}
