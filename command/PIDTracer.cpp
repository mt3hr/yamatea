#include "PIDTracer.h"
#include "ColorSensor.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "RobotAPI.h"
#include "DebugUtil.h"

using namespace ev3api;
using namespace std;

PIDTracer::PIDTracer(PIDTracerMode traceModea, int pwma, float kpa, float kia, float kda, float dta)
{
    traceMode = traceModea;
    pwm = pwma;
    kp = kpa;
    ki = kia;
    kd = kda;
    dt = dta;
}

PIDTracer::~PIDTracer()
{
}

void PIDTracer::run(RobotAPI *robotAPI)
{
    // PID制御
    brightness = robotAPI->getColorSensor()->getBrightness();

    // PID値の算出ここから
    p = brightness - targetBrightness;
    i = p * dt;
    d = (p - beforeP) / dt;
    pid = kp * p + ki * i + kd * d;
    beforeP = p;
    // PID値の算出ここまで

    // 右ライントレースか左ライントレースか
    if (traceMode == RIGHT_TRACE)
    {
        leftPower = pwm - pid;
        rightPower = pwm + pid;
    }
    else if (traceMode == LEFT_TRACE)
    {
        leftPower = pwm + pid;
        rightPower = pwm - pid;
    }

    // モータを動かす
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);

#ifdef EnablePrintPIDValues
    writeDebug("PIDTracer");
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
    writeDebug("brightness: ");
    writeDebug(brightness);
    writeEndLineDebug();
    flushDebug(TRACE, robotAPI);
#endif
}

void PIDTracer::preparation(RobotAPI *robotAPI)
{
    writeDebug("PIDTracer");
    writeEndLineDebug();
    writeDebug("kp: ");
    writeDebug(kp);
    writeEndLineDebug();
    writeDebug("ki: ");
    writeDebug(ki);
    writeEndLineDebug();
    writeDebug("kd: ");
    writeDebug(kd);
    writeEndLineDebug();
    writeDebug("dt: ");
    writeDebug(dt);
    flushDebug(DEBUG, robotAPI);
    return;
}

PIDTracer *PIDTracer::generateReverseCommand()
{
    PIDTracerMode reversedMode = LEFT_TRACE; // とりあえずね
    if (traceMode == LEFT_TRACE)
    {
        reversedMode = RIGHT_TRACE;
    }
    else if (traceMode == RIGHT_TRACE)
    {
        reversedMode = LEFT_TRACE;
    }
    return new PIDTracer(reversedMode, pwm, kp, ki, kd, dt);
}

void PIDTracer::setTargetBrightness(int t)
{
    targetBrightness = t;
}