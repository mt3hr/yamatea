#include "ClockwiseObstacleDetector.h"
#include "Stopper.h"
#include "DebugUtil.h"

// 距離センサから取得できる値がthresholdDistance以上になったタイミングで、最初のオブジェクトの距離を確定します。
ClockwiseObstacleDetector::ClockwiseObstacleDetector(int pwm, float angle, int thresholdDistance, int targetLeft, int targetRight)
{
    this->state = CODS_DETECTING_LEFT_OBSTACLE;
    this->turnWalker = new Walker(pwm, -pwm);
    this->stopper = new Stopper();
    this->pwm = pwm;
    this->angle = angle;
    this->thresholdDistance = thresholdDistance;
    this->targetLeft = targetLeft;
    this->targetRight = targetRight;
    this->reverse = false;
};

ClockwiseObstacleDetector::~ClockwiseObstacleDetector()
{
    delete turnWalker;
    delete stopper;
};

void ClockwiseObstacleDetector::run(RobotAPI *robotAPI)
{
    int currentDistance = robotAPI->getSonarSensor()->getDistance();
    int currentAngle = robotAPI->getGyroSensor()->getAngle();
#ifndef SimulatorMode
    currentAngle *= -1;
#endif

    writeDebug("currentDistance: ");
    writeDebug(currentDistance);
    writeEndLineDebug();
    writeDebug("currentAngle: ");
    writeDebug(currentAngle);
    flushDebug(TRACE, robotAPI);

    if (!reverse)
    {
        if (targetAngle < currentAngle)
        {
            state = CODS_FINISH;
            stopper->run(robotAPI);
            detectedLeftObstacleAngle = true;
            detectedRightObstacleAngle = true;
            detectedLeftObstacleDistance = true;
            detectedRightObstacleDistance = true;

            writeDebug("finishClockwiseObstacleDetector");
            flushDebug(TRACE, robotAPI);
            return;
        }
        switch (state)
        {
        case CODS_DETECTING_LEFT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            if (thresholdDistance <= currentDistance)
            {
                detectedLeftObstacleDistance = true;
                detectedLeftObstacleAngle = true;
                state = CODS_DETECTING_RIGHT_OBSTACLE;
            }
            else
            {
                leftObstacleDistance = currentDistance;
                leftObstacleAngle = float(currentAngle);
            }

            if (state != CODS_DETECTING_RIGHT_OBSTACLE)
            {
                break;
            }

            writeDebug("CODS_DETECTING_LEFT_OBSTACLE");
            flushDebug(TRACE, robotAPI);
        }
        case CODS_DETECTING_RIGHT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            if (targetLeft >= currentDistance)
            {
                state = CODS_FINISH;
                rightObstacleDistance = currentDistance;
                rightObstacleAngle = float(currentAngle);
                detectedRightObstacleDistance = true;
                detectedRightObstacleAngle = true;
            }
            if (state != CODS_FINISH)
            {
                break;
            }
            stopper->run(robotAPI);

            writeDebug("CODS_DETECTING_RIGHT_OBSTACLE");
            flushDebug(TRACE, robotAPI);
            printValues(robotAPI);
        }
        case CODS_FINISH:
        {
            break;
        }
        default:
            break;
        }
    }
    else
    {
        if (currentAngle > targetAngle)
        {
            state = CODS_FINISH;
            stopper->run(robotAPI);

            detectedLeftObstacleAngle = true;
            detectedRightObstacleAngle = true;
            detectedLeftObstacleDistance = true;
            detectedRightObstacleDistance = true;
            return;

            writeDebug("ClockwiseObstacleDetector finished");
            flushDebug(TRACE, robotAPI);
        }
        switch (state)
        {
        case CODS_DETECTING_RIGHT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            if (thresholdDistance <= currentDistance)
            {
                detectedRightObstacleDistance = true;
                detectedRightObstacleAngle = true;
                state = CODS_DETECTING_LEFT_OBSTACLE;
            }
            else
            {
                rightObstacleDistance = currentDistance;
                rightObstacleAngle = float(currentAngle);
            }

            if (state != CODS_DETECTING_LEFT_OBSTACLE)
            {
                break;
            }

            writeDebug("CODS_DETECTING_RIGHT_OBSTACLE");
            flushDebug(TRACE, robotAPI);
        }
        case CODS_DETECTING_LEFT_OBSTACLE:
        {
            turnWalker->run(robotAPI);

            if (targetRight <= currentDistance)
            {
                state = CODS_FINISH;
                leftObstacleDistance = currentDistance;
                leftObstacleAngle = float(currentAngle);
                detectedLeftObstacleDistance = true;
                detectedLeftObstacleAngle = true;
            }
            if (state != CODS_FINISH)
            {
                break;
            }
            stopper->run(robotAPI);

            writeDebug("CODS_DETECTING_LEFT_OBSTACLE");
            flushDebug(TRACE, robotAPI);
            printValues(robotAPI);
        }
        case CODS_FINISH:
        {
            break;
        }
        default:
            break;
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
    flushDebug(TRACE, robotAPI);
}

ClockwiseObstacleDetector *ClockwiseObstacleDetector::generateReverseCommand()
{
    ClockwiseObstacleDetector *reversed = new ClockwiseObstacleDetector(-pwm, -angle, thresholdDistance, targetRight, targetLeft);
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
    float currentAngle = robotAPI->getGyroSensor()->getAngle();
#ifndef SimulatorMode
    currentAngle *= -1;
#endif
    targetAngle = currentAngle + angle;
}

bool ClockwiseObstacleDetector::isFinished()
{
    return isDetectedLeftObstacleAngle() && isDetectedRightObstacleAngle() && isDetectedLeftObstacleDistance() && isDetectedRightObstacleDistance();
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
