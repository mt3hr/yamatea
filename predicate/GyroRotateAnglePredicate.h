#ifndef GyroRotateAnglePredicate_H
#define GyroRotateAnglePredicate_H

#include "Predicate.h"

// GyroRotateAnglePredicate 
// ジャイロセンサの角度が渡された値を超えたらtrueを返すPredicate。
// +で時計回り方向、-で反時計回り方向。（シミュレータと実機でジャイロセンサの挙動が異なるので注意（このPredicateでは対応済み））
// 
// 実方
class GyroRotateAnglePredicate : public Predicate
{
private:
    int angle;
    int targetAngle;
    bool clockwise;

public:
    GyroRotateAnglePredicate(int angle);
    virtual ~GyroRotateAnglePredicate();
    virtual bool test(RobotAPI *robotAPI) override;
    virtual void preparation(RobotAPI *robotAPI) override;
    virtual GyroRotateAnglePredicate *generateReversePredicate() override;
};

#endif