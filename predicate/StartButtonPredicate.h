#ifndef StartButtonPredicate_H
#define StartButtonPredicate_H

#include "Predicate.h"
#include "TouchSensor.h"
using namespace ev3api;

// StartButtonPredicate
// スタートボタン（タッチセンサ）が押されたらtrueを返すPredicate
//
// 実方
class StartButtonPredicate : public Predicate
{
private:
  TouchSensor *touchSensor;

public:
  StartButtonPredicate(TouchSensor *touchSensor);
  bool test() override;
};

#endif