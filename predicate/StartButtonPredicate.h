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
  virtual ~StartButtonPredicate();
  virtual bool test(RobotAPI *robotAPI) override;
  virtual void preparation(RobotAPI *robotAPI) override;
  virtual StartButtonPredicate *generateReversePredicate() override;
};

#endif