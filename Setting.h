#ifndef Setting_H
#define Setting_H

// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
// ********** 設定1/2ここから **********

// 実機シミュレータ設定。ジャイロセンサから取得できる角度の方向が実機とシミュレータでは異なるので。
//#define SimulatorMode // 実機で動かすときにはコメントアウトして

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
//#define LeftCourceMode // 左コース用プログラム
//#define RightCourceMode // 右コース用プログラム
//#define SlalomTestMode // スラロームだけ走行するプログラム。
#define SlalomAwaitingSignalMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。
//#define BlockTestMode  // ブロック搬入だけを走行するプログラム。
//#define FlatLineMode // すべて同じPIDで倉庫する左コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode    // RGBRawの値をはかるプログラム
//#define Rotate360TestMode // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
//#define RotateTestMode // 旋回モード。テスト用
//#define RotateGyroTestMode // ジャイロを使った旋回モード。テスト用。
//#define StraightTestMode // 直進モード。テスト用
//#define CurvatureWalkerTestMode // 曲率旋回モード。テスト用
//#define SwingSonarDetectorTestMode // 障害物距離角度首振り検出モード。テスト用
//#define ShigekiTestMode // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
//#define UFORunnerSwingTestMode // UFO走行モード。障害物間を向いている状態から始める。テスト用
//#define UFORunnerClockwiseTestMode // UFO走行モード。左障害物を向いている状態から始める。テスト用
// モード設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ

// ********** 設定1/2ここまで **********

enum DEBUG_LEVEL
{
    NONE,
    INFO,
    DEBUG,
    TRACE,
};

extern float wheelDiameter;
extern float distanceFromSonarSensorToAxle;
extern float wheelSpace;
extern int angleFor360TurnLeftRotateRobot;
extern int angleFor360TurnRightRotateRobot;
extern DEBUG_LEVEL debugMessageLevel;
extern bool enablePrintMessageMode;
extern bool enablePrintMessageForConsole;
extern bool enablePrintMessageForBluetooth;
extern bool enableBeepWhenCommandSwitching;
extern void setting();

#endif