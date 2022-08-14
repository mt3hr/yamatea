#ifndef StartButtonPredicate_H
#define StartButtonPredicate_H

#include "Predicate.h"
#include "TouchSensor.h"
#include "RobotAPI.h"

using namespace ev3api;

// StartButtonPredicate
// スタートボタン（タッチセンサ）が押されたらtrueを返すPredicate
//
// 実方
class StartButtonPredicate : public Predicate
{
private:

public:
  StartButtonPredicate();
  bool test(RobotAPI *robotAPI) override;
  void preparation(RobotAPI *robotAPI) override;
};

#endif