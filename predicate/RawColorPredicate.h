#ifndef RawColorPredicate_H
#define RawColorPredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

enum RawColorPredicateCondition
{
    IGNORE,
    GREATER_THAN,
    LESS_THAN,
    BETWEEN3,
    BETWEEN5,
    BETWEEN10,
};

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

class BluePredicate : public RawColorPredicate
{
private:
public:
    BluePredicate();
    virtual ~BluePredicate();
};

class RedPredicate : public RawColorPredicate
{
private:
public:
    RedPredicate();
    virtual ~RedPredicate();
};

class YellowPredicate : public RawColorPredicate
{
private:
public:
    YellowPredicate();
    virtual ~YellowPredicate();
};

class GreenPredicate : public RawColorPredicate
{
private:
public:
    GreenPredicate();
    virtual ~GreenPredicate();
};

#endif