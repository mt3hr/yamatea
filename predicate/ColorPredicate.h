#ifndef ColorPredicate_H
#define ColorPredicate_H

#include "Predicate.h"
#include "Sensor.h"
#include "RobotAPI.h"

// 非推奨。ColorIDを使うと青と白（青と黒だっけ？）の誤検知が多い
// 代わりにRawColorPredicateがあるのでそちらを使って
//
// ColorPredicate
// 現在のカラーセンサから取得できる値が渡されたColorIDと一致していればTrueを返すPredicate。
//
// 実方
class ColorPredicate : public Predicate
{
private:
    colorid_t colorID;

public:
    ColorPredicate(colorid_t colorID);
    virtual ~ColorPredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual Predicate *generateReversePredicate();
};

#endif