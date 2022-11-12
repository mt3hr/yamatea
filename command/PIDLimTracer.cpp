#include "PIDLimTracer.h"
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

PIDLimTracer::PIDLimTracer(PIDTracerMode traceModea, PIDLimTracerMode mode, float pwma, float ku, float tu, float dta, float r) : PIDTracer(traceModea, pwma, 0, 0, 0, dta) // スーパーコンストラクタはとりあえず0でいいや
{
    traceMode = traceModea;
    this->mode = mode;
    pwm = pwma;
    this->ku = ku;
    this->tu = tu;
    this->dt = dta;
    this->r = r;
    switch (mode)
    {
    case PLTM_P:
    {
        kp = ku * 0.5;
        ti = 0;
        td = 0;
        break;
    };
    case PLTM_PI:
    {
        kp = ku * 0.45;
        ti = tu * 0.83;
        td = 0;
        break;
    };
    case PLTM_PID:
    {
        kp = ku * 0.6;
        ti = tu * 0.5;
        td = tu * 0.125;
        break;
    };
    case PLTM_PID_MOD:
    {
        kp = ku * 0.39;
        ti = tu * 0.5;
        td = tu * 0.12;
        break;
    }
    }
}

PIDLimTracer::~PIDLimTracer()
{
}

void PIDLimTracer::run(RobotAPI *robotAPI)
{
    // PID制御
    brightness = robotAPI->getColorSensor()->getBrightness();

    // PID値の算出ここから
    p = brightness - targetBrightness;
    integral += (p + beforeP) / 2 * dt;
    i = integral;
    d = (p - beforeP) / dt;
    pid = kp * (p + 1 / ti * i + td * d);
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
    writeDebug(p * kp);
    writeEndLineDebug();
    writeDebug("i: ");
    writeDebug(i * 1 / ti);
    writeEndLineDebug();
    writeDebug("d: ");
    writeDebug(d * td);
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
    writeDebug("brightness: ");
    writeDebug(brightness);
    writeEndLineDebug();
    flushDebug(TRACE, robotAPI);
#endif
}

void PIDLimTracer::preparation(RobotAPI *robotAPI)
{
    writeDebug("PIDTracer");
    writeEndLineDebug();
    writeDebug("kp: ");
    writeDebug(kp);
    writeEndLineDebug();
    writeDebug("ti: ");
    writeDebug(ti);
    writeEndLineDebug();
    writeDebug("td: ");
    writeDebug(td);
    writeEndLineDebug();
    writeDebug("dt: ");
    writeDebug(dt);
    writeEndLineDebug();
    writeDebug("target brightness: ");
    writeDebug(targetBrightness);
    flushDebug(DEBUG, robotAPI);
    return;
}

PIDLimTracer *PIDLimTracer::generateReverseCommand()
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
    return new PIDLimTracer(reversedMode, mode, pwm, ku, tu, dt, r); // TODO rって反転しなくていいの？反転すると動かないんだよね
}

void PIDLimTracer::setTargetBrightness(int8_t t)
{
    targetBrightness = t;
}