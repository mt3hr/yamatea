#ifndef SuitableForRightCourse_H
#define SuitableForRightCourse_H

#include "PIDTracer.h"
#include "Walker.h"
#include "RobotAPI.h"
#include "Setting.h"

using namespace ev3api;

// PIDTracer反転関数。
// 左コースならそれをそのまま、右コースならば反転させたPIDTracerを返す
// 
// 実方
PIDTracer *ifRightThenReverseCommand(PIDTracer *pidTracer);

// Walker反転関数。
// 左コースならそれをそのまま、右コースならば反転させたWalkerを返す
// 
// 実方
Walker *ifRightThenReverseCommand(Walker *walker);

// Predicate生成関数。
// 左コースならば左車輪回転数Predicateを、
// 右コースならば右車輪回転数Predicateを生成する。
// 
// 実方
MotorCountPredicate *generateWheelCountPredicate(int count);

#endif