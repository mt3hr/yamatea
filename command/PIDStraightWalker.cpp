#include "PIDStraightWalker.h"
#include "RobotAPI.h"

PIDStraightWalker::PIDStraightWalker(int pwm, float kp, float ki, float kd, float dt)
{
    this->pwm = pwm;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->back = pwm < 0;
}

PIDStraightWalker::~PIDStraightWalker()
{
}

void PIDStraightWalker::run(RobotAPI *robotAPI)
{
    // PID制御
    wheelDifference = robotAPI->getLeftWheel()->getCount() - robotAPI->getRightWheel()->getCount();

    p = wheelDifference - trueTargetDifference;
    i = p * dt;
    d = (p - beforeP) / dt;
    pid = kp * p + ki * i + kd * d;
    beforeP = p;
    // PID値の算出ここまで

    leftPower = pwm - pid;
    rightPower = pwm + pid;

    // モータを動かす
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);

#ifdef EnablePrintPIDValues
    writeDebug("PIDStraightWalker");
    writeEndLineDebug();
    writeDebug("p: ");
    writeDebug(p);
    writeEndLineDebug();
    writeDebug("i: ");
    writeDebug(i);
    writeEndLineDebug();
    writeDebug("d: ");
    writeDebug(d);
    writeEndLineDebug();
    writeDebug("leftPow: ");
    writeDebug(leftPower);
    writeEndLineDebug();
    writeDebug("rightPow: ");
    writeDebug(rightPower);
    writeEndLineDebug();
    writeDebug("difference: ");
    writeDebug(wheelDifference);
    writeEndLineDebug();
    flushDebug(TRACE, robotAPI);
#endif
}

void PIDStraightWalker::preparation(RobotAPI *robotAPI)
{
    if (!inited)
    {
        trueTargetDifference = robotAPI->getLeftWheel()->getCount() - robotAPI->getRightWheel()->getCount() - targetDifference;
        inited = true;
    }
}

PIDStraightWalker *PIDStraightWalker::generateReverseCommand()
{
    return new PIDStraightWalker(pwm, kp, ki, kd, dt);
}

void PIDStraightWalker::setTargetDifferenceWheelCount(int targetDifference)
{
    this->targetDifference = targetDifference;
}
