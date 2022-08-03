#include "PIDTracer.h"
#include "ColorSensor.h"
#include "WheelController.h"
#include "util.h"

using namespace ev3api;

PIDTracer::PIDTracer(PIDTracerMode modea, int pwma, float kpa, float kia, float kda, float dta, int targetBrightnessa, WheelController *wheelControllera, ColorSensor *colorSensora)
{
    mode = modea;
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
    int bright = colorSensor->getBrightness();

    // PID値の算出ここから
    float p = bright - targetBrightness;
    float i = p * dt;
    float d = (p - beforeP) / dt;
    float pid = kp * p + ki * i + kd * d;
    beforeP = p;
    // PID値の算出ここまで

    // 右ライントレースか左ライントレースか
    int leftPower = 0;
    int rightPower = 0;
    if (mode == RIGHT_TRACE)
    {
        leftPower = pwm - pid;
        rightPower = pwm + pid;
    }
    else if (mode == LEFT_TRACE)
    {
        leftPower = pwm + pid;
        rightPower = pwm - pid;
    }

    // モータを動かす
    wheelController->getLeftWheel()->setPWM(leftPower);
    wheelController->getRightWheel()->setPWM(rightPower);

#ifdef PrintMessage
    char pStr[30];
    char iStr[30];
    char dStr[30];
    char lStr[30];
    char rStr[30];
    sprintf(pStr, "p:%.2lf", p);
    sprintf(iStr, "i:%.2lf", i);
    sprintf(dStr, "d:%.2lf", d);
    sprintf(lStr, "leftPow :%d", leftPower);
    sprintf(rStr, "rightPow:%d", rightPower);
    msg_f("pid tracing", 1);
    msg_f(pStr, 2);
    msg_f(iStr, 3);
    msg_f(dStr, 4);
    msg_f(lStr, 5);
    msg_f(rStr, 6);
    msg_f("", 7);
#endif
}

PIDTracer *PIDTracer::generateReverseCommand()
{
    PIDTracerMode reversedMode = LEFT_TRACE; // とりあえずね
    if (mode == LEFT_TRACE)
    {
        reversedMode = RIGHT_TRACE;
    }
    else if (mode == RIGHT_TRACE)
    {
        reversedMode = LEFT_TRACE;
    }
    return new PIDTracer(reversedMode, pwm, kp, ki, kd, dt, targetBrightness, wheelController, colorSensor);
}

void PIDTracer::setTargetBrightness(int t)
{
    targetBrightness = t;
}