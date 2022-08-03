#ifndef SuitableForRightCourse_H
#define SuitableForRightCourse_H

#include "PIDTracer.h"
#include "Walker.h"
#include "WheelController.h"

using namespace ev3api;

// PIDTracer反転関数。
// 左コースならそれをそのまま、右コースならば反転させたPIDTracerを返す
// 
// 実方
PIDTracer *ifRightThenReverseCommand(PIDTracer *pidTracer, bool isRightCource);

// Walker反転関数。
// 左コースならそれをそのまま、右コースならば反転させたWalkerを返す
// 
// 実方
Walker *ifRightThenReverseCommand(Walker *walker, bool isRightCource);

// Predicate生成関数。
// isRightCourseがtrueならば右コース用の左車輪回転数Predicateを、
// falseならば左コース用の右車輪回転数Predicateを生成する。
// 
// 実方
MotorCountPredicate *generateMotorCountPredicate(bool isRightCource, int count, WheelController *wheelController);

#endif