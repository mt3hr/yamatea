#include "PIDTracer.h"
#include "ColorSensor.h"
#include "WheelController.h"
#include "PrintMessage.h"
#include "Setting.h"
#include "string"
#include "vector"
#include "sstream"
#include "iomanip"

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

    // メッセージ出力処理。変数名は雑。
    if (enablePrintMessageMode)
    {
        stringstream ps;
        stringstream is;
        stringstream ds;
        stringstream ls;
        stringstream rs;
        stringstream bs;

        ps.clear();
        is.clear();
        ds.clear();
        ls.clear();
        rs.clear();
        bs.clear();

        ps.str("");
        is.str("");
        ds.str("");
        ls.str("");
        rs.str("");
        bs.str("");

        ps << fixed;
        is << fixed;
        ds << fixed;

        ps << "p:" << setprecision(2) << p;
        is << "i:" << setprecision(2) << i;
        ds << "d:" << setprecision(2) << d;
        ls << "leftPow   :" << float(leftPower);  // intのままだと出力されないのでfloatに変換する
        rs << "rightPow  :" << float(rightPower); // intのままだと出力されないのでfloatに変換する
        bs << "brightness:" << float(brightness); // intのままだと出力されないのでfloatに変換する

        vector<string> messageLines;
        messageLines.push_back("pid tracing");
        messageLines.push_back(ps.str());
        messageLines.push_back(is.str());
        messageLines.push_back(ds.str());
        messageLines.push_back(ls.str());
        messageLines.push_back(rs.str());
        messageLines.push_back(bs.str());
        PrintMessage printMessage(messageLines, false);
        printMessage.run();
    }
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