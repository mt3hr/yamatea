#include "PIDTracer.h"
#include "ColorSensor.h"
#include "WheelController.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "DebugUtil.h"

using namespace ev3api;
using namespace std;

PIDTracer::PIDTracer(PIDTracerMode traceModea, int pwma, float kpa, float kia, float kda, float dta, int targetBrightnessa, WheelController *wheelControllera, ColorSensor *colorSensora)
{
    traceMode = traceModea;
    pwm = pwma;
    kp = kpa;
    ki = kia;
    kd = kda;
    dt = dta;
    targetBrightness = targetBrightnessa;
    wheelController = wheelControllera;
    colorSensor = colorSensora;
}

void PIDTracer::run()
{
    // PID制御
    brightness = colorSensor->getBrightness();

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
    wheelController->getLeftWheel()->setPWM(leftPower);
    wheelController->getRightWheel()->setPWM(rightPower);

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
    flushDebug();
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
    return new PIDTracer(reversedMode, pwm, kp, ki, kd, dt, targetBrightness, wheelController, colorSensor);
}

void PIDTracer::setTargetBrightness(int t)
{
    targetBrightness = t;
}