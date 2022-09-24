#ifndef SonarDistancePredicate_H
#define SonarDistancePredicate_H

#include "Predicate.h"
#include "RobotAPI.h"

// SonarDistancePredicate
// ソナーセンサで指定距離（cm）以下の数値が取得できたらtrueを返すPredicate。
// ある距離を確実に保ちたいならば、command/Hedgehogクラスを使った方がいい
//
// 実方
class SonarDistancePredicate : public Predicate
{
private:
    int distance;
    bool lessThan;

public:
    SonarDistancePredicate(int distance, bool lessThan);
    virtual ~SonarDistancePredicate();
    virtual bool test(RobotAPI *robotAPI);
    virtual void preparation(RobotAPI *robotAPI);
    virtual SonarDistancePredicate *generateReversePredicate();
};

#endif