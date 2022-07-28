#include "PIDTracer.h"
#include "ColorSensor.h"

using namespace ev3api;

PIDTracer::PIDTracer(PIDTracerMode modea, int pwma, float kpa, float kia, float kda, float dta, int targetBrightnessa, Motor *leftMotora, Motor *rightMotora, ColorSensor *colorSensora)
{
    mode = modea;
    pwm = pwma;
    kp = kpa;
    ki = kia;
    kd = kda;
    dt = dta;
    targetBrightness = targetBrightnessa;
    leftWheel = leftMotora;
    rightWheel = rightMotora;
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
    float leftPower = 0;
    float rightPower = 0;
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
    leftWheel->setPWM(leftPower);
    rightWheel->setPWM(rightPower);
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
    return new PIDTracer(reversedMode, pwm, kp, ki, kd, dt, targetBrightness, leftWheel, rightWheel, colorSensor);
}

void PIDTracer::setTargetBrightness(int t)
{
    targetBrightness = t;
}