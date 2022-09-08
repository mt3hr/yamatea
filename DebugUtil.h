#ifndef DebugUtil_H
#define DebugUtil_H

#include "string"
#include "RobotAPI.h"
#include "Setting.h"

using namespace std;

// デバッグ用出力関数集。
// メッセージ入力  : writeDebug(message)して、
// 必要に応じて改行: writeEndLineDebug()して、
// 出力            : flushDebug()する。

//NOTE streamを自作すればもっと楽にデバッグできるんだけどね。まあいいや。

void clearDebug();
void writeDebug(string str);
void writeDebug(int i);
void writeDebug(float f);
void writeDebug(uint64_t i);
void writeDebug(int32_t i);
void writeEndLineDebug();
void flushDebug(DEBUG_LEVEL level, RobotAPI *robotAPI);
void writeAndFlushDebug(string str, DEBUG_LEVEL level, RobotAPI *robotAPI);
void beepDebug();

// 変数名取得
#define GET_VARIABLE_NAME(Variable) (#Variable)

#endif