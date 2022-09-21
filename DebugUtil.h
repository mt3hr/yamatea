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
// 
// 実方

// NOTE streamを自作すればもっと楽にデバッグできるんだけどね。まあいいや。

// デバッグダンプをクリアする。
void clearDebug();

// デバッグメッセージを書き込む
void writeDebug(string str);

// デバッグメッセージを書き込む
void writeDebug(int i);

// デバッグメッセージを書き込む
void writeDebug(float f);

// デバッグメッセージを書き込む
void writeDebug(uint16_t i);

// デバッグメッセージを書き込む
void writeDebug(uint64_t i);

// デバッグメッセージを書き込む
void writeDebug(int32_t i);

// デバッグメッセージを書き込む
void writeDebug(uint16_t i);

// デバッグメッセージを書き込む
void writeDebug(int32_t i);

// デバッグメッセージを改行する
void writeEndLineDebug();

// デバッグメッセージを出力する
void flushDebug(DEBUG_LEVEL level, RobotAPI *robotAPI);

// デバッグメッセージを書き込み、出力する
void writeAndFlushDebug(string str, DEBUG_LEVEL level, RobotAPI *robotAPI);

// ビープ音を鳴らす
void beepDebug();

// LEDを切り替える
void ledDebug();

// 変数名取得
#define GET_VARIABLE_NAME(Variable) (#Variable)

#endif