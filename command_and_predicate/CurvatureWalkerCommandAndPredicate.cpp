#include "CurvatureWalkerCommandAndPredicate.h"
#include "math.h"
#include "Setting.h"
#include "Walker.h"
#include "DistancePredicate.h"
#include "DebugUtil.h"

CurvatureWalkerCommandAndPredicate::CurvatureWalkerCommandAndPredicate(int pwm, float r, float theta, RobotAPI *robotAPI)
{
    float wheelSpaceDivide2 = wheelSpace / 2; // 先に割り算しておかないとシミュレータで期待どおりに動かない
    // 時計回りの時。
    // 中央の孤の長さ: lone  = 半径 * rad(角度)
    // 左の孤の長さ  : loneL = (半径 + (左車輪右車輪の間隔 / 2)) * rad(角度)
    // 右の孤の長さ  : loneR = (半径 - (左車輪右車輪の間隔 / 2)) * rad(角度)
    float lone;
    float loneL;
    float loneR;
    if (theta > 0)
    {
        lone = r * (theta * M_PI / 180);
        loneL = (r + wheelSpaceDivide2) * (theta * M_PI / 180);
        loneR = (r - wheelSpaceDivide2) * (theta * M_PI / 180);
    }
    else
    {
        lone = r * (-theta * M_PI / 180);
        loneL = (r - wheelSpaceDivide2) * (-theta * M_PI / 180);
        loneR = (r + wheelSpaceDivide2) * (-theta * M_PI / 180);
    }

    float ratioL = loneL / lone;
    float ratioR = loneR / lone;
    int leftPWM = pwm * ratioL;
    int rightPWM = pwm * ratioR;

    writeDebug("leftPWM");
    writeDebug(leftPWM);
    writeEndLineDebug();
    writeDebug("rightPWM");
    writeDebug(rightPWM);
    writeEndLineDebug();
    flushDebug(DEBUG, robotAPI);

    Command *walker = new Walker(leftPWM, rightPWM);

    DistancePredicate *predicate = new DistancePredicate(loneL, robotAPI);
    if (theta < 0)
    {
        predicate = predicate->generateReversePredicate();
    }

    setCommand(walker);
    setPredicate(predicate);
}

CurvatureWalkerCommandAndPredicate::~CurvatureWalkerCommandAndPredicate()
{
}