#ifndef AngleAbsPredicate_H
#define AngleAbsPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

enum AngleAbsPredicateMode
{
    AAPM_Gyro,
    AAPM_WheelCount,
};

// ジャイロセンサや車輪回転数から導き出される角度が、その値以上になったらtrueを返すPredicate。
//
// 実方
class AngleAbsPredicate : public Predicate
{
private:
    AngleAbsPredicateMode mode;
    int angle;
    bool up;

public:
    AngleAbsPredicate(AngleAbsPredicateMode mode, int angle, bool up);
    virtual ~AngleAbsPredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual AngleAbsPredicate *generateReversePredicate();
};

#endif