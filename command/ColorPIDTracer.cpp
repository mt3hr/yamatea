#include "ColorPIDTracer.h"
#include "PIDTracer.h"
#include "ColorSensor.h"
#include "Setting.h"
#include "string"
#include "iomanip"
#include "RobotAPI.h"
#include "DebugUtil.h"

using namespace ev3api;
using namespace std;

ColorPIDTracer::ColorPIDTracer(PIDTracerMode traceModea, TraceColor traceColor, int pwma, float kpa, float kia, float kda, float dta)
{
    traceMode = traceModea;
    this->traceColor = traceColor;
    pwm = pwma;
    kp = kpa;
    ki = kia;
    kd = kda;
    dt = dta;
}

ColorPIDTracer::~ColorPIDTracer()
{
}

void ColorPIDTracer::run(RobotAPI *robotAPI)
{
    // PID制御
    rgb_raw_t rgbRaw;
    robotAPI->getColorSensor()->getRawColor(rgbRaw);
    rgb = rgbRaw;

    // PID値の算出ここから
    switch (traceColor)
    {
    case Trace_R:
    {
        p = rgb.r - targetRGB.r;
        break;
    }
    case Trace_G:
    {
        p = rgb.g - targetRGB.g;
        break;
    }
    case Trace_B:
    {
        p = rgb.b - targetRGB.b;
        break;
    }
    default:
        p = rgb.r - targetRGB.r;
        break;
    }
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
    writeDebug("ColorPIDTracer");
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
    writeDebug("r: ");
    writeDebug(rgb.r);
    writeEndLineDebug();
    writeDebug("g: ");
    writeDebug(rgb.g);
    writeEndLineDebug();
    writeDebug("b: ");
    writeDebug(rgb.b);
    writeEndLineDebug();
    flushDebug(TRACE, robotAPI);
#endif
}

void ColorPIDTracer::preparation(RobotAPI *robotAPI)
{
    writeDebug("ColorPIDTracer");
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
    writeDebug("target color");
    writeEndLineDebug();
    writeDebug("r: ");
    writeDebug(targetRGB.r);
    writeEndLineDebug();
    writeDebug("g: ");
    writeDebug(targetRGB.g);
    writeEndLineDebug();
    writeDebug("b: ");
    writeDebug(targetRGB.b);
    flushDebug(DEBUG, robotAPI);
    return;
}

ColorPIDTracer *ColorPIDTracer::generateReverseCommand()
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
    return new ColorPIDTracer(reversedMode, traceColor, pwm, kp, ki, kd, dt);
}

void ColorPIDTracer::setTargetColor(rgb_raw_t targetRGB)
{
    this->targetRGB = targetRGB;
}
