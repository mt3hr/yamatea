#ifndef RawColorPredicate_H
#define RawColorPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

// TODO BETWEEN8ほしくない？？

// RawColorPredicateCOndition
// RawColorPredicateの条件。
//
// 実方
enum RawColorPredicateCondition
{
    IGNORE,       // 無視する。常にtrueを返す
    GREATER_THAN, // 値が大きければtrueを返す
    LESS_THAN,    // 値が小さければtrueを返す
    BETWEEN3,     // 値が+-3の範囲内ならtrueを返す
    BETWEEN5,     // 値が+-5の範囲内ならtrueを返す
    BETWEEN10,    // 値が+-10の範囲内ならtrueを返す
    BETWEEN15,    // 値が+-15の範囲内ならtrueを返す
    BETWEEN20,    // 値が+-20の範囲内ならtrueを返す
};

// RawColorPredicate
// 現在のカラーセンサから取得できる値が渡されたrawColorの範囲内ならばTrueを返すPredicate。
//
// 実方
class RawColorPredicate : public Predicate
{
private:
    int *r;
    int *g;
    int *b;
    RawColorPredicateCondition rCondition;
    RawColorPredicateCondition gCondition;
    RawColorPredicateCondition bCondition;

public:
    RawColorPredicate(int *r, RawColorPredicateCondition rCondition, int *g, RawColorPredicateCondition gCondition, int *b, RawColorPredicateCondition bCondition);
    virtual ~RawColorPredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual Predicate *generateReversePredicate() override;
};

// 現在のカラーセンサから取得できる値がキャリブレーションした青の範囲内であればTrueを返すPredicate。
class BluePredicate : public RawColorPredicate
{
private:
public:
    BluePredicate();
    virtual ~BluePredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションした赤の範囲内であればTrueを返すPredicate。
class RedPredicate : public RawColorPredicate
{
private:
public:
    RedPredicate();
    virtual ~RedPredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションした黄の範囲内であればTrueを返すPredicate。
class YellowPredicate : public RawColorPredicate
{
private:
public:
    YellowPredicate();
    virtual ~YellowPredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションした緑の範囲内であればTrueを返すPredicate。
class GreenPredicate : public RawColorPredicate
{
private:
public:
    GreenPredicate();
    virtual ~GreenPredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションした青白エッジの範囲内であればTrueを返すPredicate。
class BlueEdgePredicate : public RawColorPredicate
{
private:
public:
    BlueEdgePredicate();
    virtual ~BlueEdgePredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションした黒の範囲内であればTrueを返すPredicate。
class BlackPredicate : public RawColorPredicate
{
private:
public:
    BlackPredicate();
    virtual ~BlackPredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションしたスラロームから見た白の範囲内であればTrueを返すPredicate。
class WhiteAtSlaromPredicate : public RawColorPredicate
{
private:
public:
    WhiteAtSlaromPredicate();
    virtual ~WhiteAtSlaromPredicate();
};

// 現在のカラーセンサから取得できる値がキャリブレーションした灰の範囲内であればTrueを返すPredicate。
class GrayPredicate : public RawColorPredicate
{
private:
public:
    GrayPredicate();
    virtual ~GrayPredicate();
};

#endif