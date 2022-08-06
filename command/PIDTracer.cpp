#include "PIDTracer.h"
#include "ColorSensor.h"
#include "WheelController.h"
#include "string"

using namespace ev3api;
using namespace std;

PIDTracer::~PIDTracer()
{
    delete printMessage;
}

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
    string messageLines[] = {"pid trace started"};
    printMessage = new PrintMessage(messageLines);
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

    // メッセージ出力処理
    char pStr[30];
    char iStr[30];
    char dStr[30];
    char lStr[30];
    char rStr[30];
    char bStr[30];
    sprintf(pStr, "p:%.2lf\r\n", p);
    sprintf(iStr, "i:%.2lf\r\n", i);
    sprintf(dStr, "d:%.2lf\r\n", d);
    sprintf(lStr, "leftPow :%d\r\n", leftPower);
    sprintf(rStr, "rightPow:%d\r\n", rightPower);
    sprintf(bStr, "brightness:%d\r\n", bright);

    string messageLines[] = {
        string("pid tracing\r\n"),
        string(pStr),
        string(iStr),
        string(dStr),
        string(lStr),
        string(rStr),
        string(bStr),
    };
    printMessage->setMessageLines(messageLines);
    printMessage->run();
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