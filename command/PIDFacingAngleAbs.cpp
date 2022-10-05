#include "PIDFacingAngleAbs.h"
#include "RobotAPI.h"
#include "Walker.h"
#include "Stopper.h"
#include "DebugUtil.h"
#include "FinishConfirmable.h"

PIDFacingAngleAbs::PIDFacingAngleAbs(FacingAngleMode mode, int targetAngle, float kp, float ki, float kd, float dt)
{
    this->mode = mode;
    this->targetAngle = targetAngle;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
};

PIDFacingAngleAbs::~PIDFacingAngleAbs(){};

void PIDFacingAngleAbs::run(RobotAPI *robotAPI)
{
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
    writeEndLineDebug();
    writeDebug("targetAngle: ");
    writeDebug(targetAngle);
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
    writeDebug("PIDPIDFacingAngle");
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

void PIDFacingAngleAbs::preparation(RobotAPI *robotAPI)
{
    return;
}

PIDFacingAngleAbs *PIDFacingAngleAbs::generateReverseCommand()
{
    return new PIDFacingAngleAbs(mode, -targetAngle, kp, ki, kd, dt);
}

bool PIDFacingAngleAbs::isFinished()
{
    return finish;
}
