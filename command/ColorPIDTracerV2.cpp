#include "ColorPIDTracerV2.h"
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

ColorPIDTracerV2::ColorPIDTracerV2(PIDTracerMode traceModea, TraceColor traceColor, float pwma, float kpa, float kia, float kda, float dta, float r) : ColorPIDTracer(traceModea, traceColor, pwm, kp, ki, kd, dt)
{
    traceMode = traceModea;
    this->traceColor = traceColor;
    pwm = pwma;
    kp = kpa;
    ki = kia;
    kd = kda;
    dt = dta;
    this->r = r;
}

ColorPIDTracerV2::~ColorPIDTracerV2()
{
}

void ColorPIDTracerV2::run(RobotAPI *robotAPI)
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
    integral += (p + beforeP) / 2 * dt;
    i = integral;
    d = (p - beforeP) / dt;
    pid = kp * p + ki * i + kd * d;
    beforeP = p;
    // PID値の算出ここまで

    // 右ライントレースか左ライントレースか
    float r2 = r / 2;
    if (traceMode == RIGHT_TRACE)
    {
        leftPower = (pwm - pid + r2);
        rightPower = (pwm + pid - r2);
    }
    else if (traceMode == LEFT_TRACE)
    {
        leftPower = (pwm + pid - r2);
        rightPower = (pwm - pid + r2);
    }

    // モータを動かす
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);

#ifdef EnablePrintPIDValues
    writeDebug("ColorPIDTracerV2");
    writeEndLineDebug();
    writeDebug("p: ");
    writeDebug(p * kp);
    writeEndLineDebug();
    writeDebug("i: ");
    writeDebug(i * ki);
    writeEndLineDebug();
    writeDebug("d: ");
    writeDebug(d * kd);
    writeEndLineDebug();
    writeDebug("r: ");
    writeDebug(r);
    writeEndLineDebug();
    writeDebug("pid: ");
    writeDebug(pid);
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

void ColorPIDTracerV2::preparation(RobotAPI *robotAPI)
{
    writeDebug("ColorPIDTracerV2");
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

ColorPIDTracerV2 *ColorPIDTracerV2::generateReverseCommand()
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
    return new ColorPIDTracerV2(reversedMode, traceColor, pwm, kp, ki, kd, dt, -r);
}

void ColorPIDTracerV2::setTargetColor(rgb_raw_t targetRGB)
{
    this->targetRGB = targetRGB;
}
