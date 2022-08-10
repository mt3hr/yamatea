#include "CurvatureWalker.h"
#include "math.h"
#include "Setting.h"
#include "Walker.h"
#include "WheelController.h"
#include "DistancePredicate.h"
#include "ExecutePreparationWhenExitBeforeCommandHandler.h"

// clock = trueで時計回り //TODO クソ実装では？
CommandAndPredicate *generateCurvatureWalkerWithTheta(int pwm, float r, float theta, bool clock, WheelController *wheelController)
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
    Command *walker = new Walker(leftPWM, rightPWM, wheelController);

    DistancePredicate *predicate;
    if (clock)
    {
        predicate = new DistancePredicate(loneL, wheelController->getLeftWheel());
    }
    else
    {
        predicate = new DistancePredicate(loneR, wheelController->getRightWheel());
    }

    Handler *preHandler = new ExecutePreparationWhenExitBeforeCommandHandler(predicate);

    return new CommandAndPredicate(walker, predicate, preHandler);
}
