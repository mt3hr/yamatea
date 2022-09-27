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

float integralForColorPIDTracer = 0;

ColorPIDTracerV2::ColorPIDTracerV2(PIDTracerMode traceModea, TraceColor traceColor, int pwma, float kpa, float kia, float kda, float dta, float r) : ColorPIDTracer(traceModea, traceColor, pwm, kp, ki, kd, dt)
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
    bool integralIsZero = integralForColorPIDTracer == 0;
    integralForColorPIDTracer += (p + beforeP) / 2 * dt;
    if (!integralIsZero)
    {
        i = integralForColorPIDTracer;
    }
    d = (p - beforeP) / dt;
    pid = kp * p + ki * i + kd * d;
    pidr = pid + r;
    beforeP = p;
    // PID値の算出ここまで

    // 右ライントレースか左ライントレースか
    if (traceMode == RIGHT_TRACE)
    {
        leftPower = round(pwm - pidr);
        rightPower = round(pwm + pidr);
    }
    else if (traceMode == LEFT_TRACE)
    {
        leftPower = round(pwm + pidr);
        rightPower = round(pwm - pidr);
    }

    // モータを動かす
    robotAPI->getLeftWheel()->setPWM(leftPower);
    robotAPI->getRightWheel()->setPWM(rightPower);

#ifdef EnablePrintPIDValues
    writeDebug("ColorPIDTracerV2");
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
    writeDebug("pid: ");
    writeDebug(pid);
    writeEndLineDebug();
    writeDebug("pidr: ");
    writeDebug(pidr);
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
