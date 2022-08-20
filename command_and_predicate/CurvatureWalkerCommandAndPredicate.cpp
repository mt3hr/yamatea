#include "CurvatureWalkerCommandAndPredicate.h"
#include "math.h"
#include "Setting.h"
#include "Walker.h"
#include "DistancePredicate.h"

// clock = trueで時計回り //TODO クソ実装では？
CurvatureWalkerCommandAndPredicate::CurvatureWalkerCommandAndPredicate(int pwm, float r, float theta, bool clock, RobotAPI *robotAPI)
{
    // 時計回りの時。
    // 中央の孤の長さ: lone  = 半径 * rad(角度)
    // 左の孤の長さ  : loneL = (半径 + (左車輪右車輪の間隔 / 2)) * rad(角度)
    // 右の孤の長さ  : loneR = (半径 - (左車輪右車輪の間隔 / 2)) * rad(角度)
    float lone = r * (theta * M_PI / float(180));
    float loneL;
    float loneR;
    if (clock)
    {
        loneL = (r + (wheelSpace / float(2))) * (theta * M_PI / float(180));
        loneR = (r - (wheelSpace / float(2))) * (theta * M_PI / float(180));
    }
    else
    {
        loneL = (r - (wheelSpace / float(2))) * (theta * M_PI / float(180));
        loneR = (r + (wheelSpace / float(2))) * (theta * M_PI / float(180));
    }

    float ratioL = loneL / lone;
    float ratioR = loneR / lone;
    int leftPWM = pwm * ratioL;
    int rightPWM = pwm * ratioR;
    Command *walker = new Walker(leftPWM, rightPWM);

    DistancePredicate *predicate = new DistancePredicate(loneL, robotAPI);
    if (!clock)
    {
        predicate = predicate->generateReversePredicate();
    }

    setCommand(walker);
    setPredicate(predicate);
}
