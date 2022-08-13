#ifndef DebugUtil_H
#define DebugUtil_H
#include "string"
#include "RobotAPI.h"

// デバッグ用出力関数集。
// メッセージ入力  : writeDebug(message)して、
// 必要に応じて改行: writeEndLineDebug()して、
// 出力            : flushDebug()する。

using namespace std;

void writeDebug(string str);
void writeDebug(int i);
void writeDebug(float f);
void writeEndLineDebug();
void flushDebug(RobotAPI *robotAPI);

#endif