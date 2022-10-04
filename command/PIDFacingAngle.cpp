#include "PIDFacingAngle.h"
#include "FacingAngleAbs.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"
#include "FinishConfirmable.h"

PIDFacingAngle::PIDFacingAngle(FacingAngleMode mode, int angle, float kp, float ki, float kd, float dt)
{
    this->mode = mode;
    this->angle = angle;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
};

PIDFacingAngle::~PIDFacingAngle(){};

void PIDFacingAngle::run(RobotAPI *robotAPI)
{
    int currentAngle = 0;
    int currentAnglef = 0;
    switch (mode)
    {
    case FA_Gyro:
    {
#ifndef SimulatorMode
        currentAngle = robotAPI->getGyroSensor()->getAngle() * -1;
        currentAnglef = robotAPI->getGyroSensor()->getAngle() * -1;
#else
        currentAngle = robotAPI->getGyroSensor()->getAngle();
        currentAnglef = robotAPI->getGyroSensor()->getAngle();
#endif
        break;
    }
    case FA_WheelCount:
    {
        currentAngle = robotAPI->getMeasAngle()->getAngle();
        currentAnglef = robotAPI->getMeasAngle()->getAngle();
        break;
    }
    default:
        break;
    }

    writeDebug("PIDFacingAngle");
    writeEndLineDebug();
    writeDebug("angle: ");
    writeDebug(currentAngle);
    flushDebug(TRACE, robotAPI);

    if (currentAngle == targetAngle)
    {
        finish = true;
        Stopper *stopper = new Stopper();
        stopper->run(robotAPI);
        delete stopper;
    }

    p = currentAnglef - targetAngle;
    integral += (p + beforeP) / 2 * dt;
    i = integral;
    d = (p - beforeP) / dt;
    pid = kp * p + ki * i + kd * d;
    beforeP = p;
    // PID値の算出ここまで

    leftPower = -pid;
    rightPower = pid;

    // モータを動かす
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);

#ifdef EnablePrintPIDValues
    writeDebug("PIDFacingAngle");
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
    flushDebug(TRACE, robotAPI);
#endif
}

void PIDFacingAngle::preparation(RobotAPI *robotAPI)
{
    int currentAngle = 0;
    switch (mode)
    {
    case FA_Gyro:
    {
#ifndef SimulatorMode
        currentAngle = robotAPI->getGyroSensor()->getAngle() * -1;
#else
        currentAngle = robotAPI->getGyroSensor()->getAngle();
#endif
        break;
    }
    case FA_WheelCount:
    {
        currentAngle = robotAPI->getMeasAngle()->getAngle();
        break;
    }
    default:
        break;
    }
    targetAngle = currentAngle + angle;
    return;
}

PIDFacingAngle *PIDFacingAngle::generateReverseCommand()
{
    return new PIDFacingAngle(mode, angle, kp, ki, kd, dt);
}

bool PIDFacingAngle::isFinished()
{
    return finish;
}
