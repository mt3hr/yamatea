#include "PIDTracerV3.h"
#include "PIDTracer.h"
#include "ColorSensor.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "RobotAPI.h"
#include "DebugUtil.h"
#include "math.h"

using namespace ev3api;
using namespace std;

double integralBrightness = 0;

PIDTracerV3::PIDTracerV3(PIDTracerMode traceModea, float pwma, float kpa, float kia, float kda, float dta, float r) : PIDTracer(traceModea, pwma, kpa, kia, kda, dta)
{
    traceMode = traceModea;
    pwm = pwma;
    kp = kpa;
    ki = kia;
    kd = kda;
    dt = dta;
    this->r = r;
}

PIDTracerV3::~PIDTracerV3()
{
}

void PIDTracerV3::run(RobotAPI *robotAPI)
{
    // PID制御
    brightness = robotAPI->getColorSensor()->getBrightness();

    // PID値の算出ここから
    p = brightness - targetBrightness;
    integralBrightness += (p + beforeP) / 2 * dt;
    i = integralBrightness;
    d = (p - beforeP) / dt;
    pid = kp * p + ki * i + kd * d;
    beforeP = p;
    // PID値の算出ここまで

    // 右ライントレースか左ライントレースか
    if (traceMode == RIGHT_TRACE)
    {
        leftPower = round(pwm - pid + r / 2);
        rightPower = round(pwm + pid - r / 2);
    }
    else if (traceMode == LEFT_TRACE)
    {
        leftPower = round(pwm + pid - r / 2);
        rightPower = round(pwm - pid + r / 2);
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
    writeDebug("r: ");
    writeDebug(r);
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

void PIDTracerV3::preparation(RobotAPI *robotAPI)
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
    writeEndLineDebug();
    writeDebug("target brightness: ");
    writeDebug(targetBrightness);
    flushDebug(DEBUG, robotAPI);
    return;
}

PIDTracerV3 *PIDTracerV3::generateReverseCommand()
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
    return new PIDTracerV3(reversedMode, pwm, kp, ki, kd, dt, -r);
}

void PIDTracerV3::setTargetBrightness(int8_t t)
{
    targetBrightness = t;
}