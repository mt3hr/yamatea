#ifndef Setting_H
#define Setting_H
#include "Note.h"
#include "RawColorPredicate.h"
#include "ev3api.h"
#include "vector"

using namespace std;

// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
// ********** 設定1/2ここから **********

// 実機シミュレータ設定。ジャイロセンサから取得できる角度の方向が実機とシミュレータでは異なるので。
//#define SimulatorMode // 実機で動かすときにはコメントアウトして

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
#define TrueLeftCourceMode // 左コース用プログラム
//#define TrueRightCourceMode // 右コース用プログラム
//#define LeftCourceMode // 左コース用プログラム
//#define RightCourceMode // 右コース用プログラム
//#define LeftCourceOkiharaMode1 // 左コース用プログラム沖原コードシナリオ追加前安定版
//#define LeftCourceOkiharaMode2 // 左コース用プログラム沖原コードシナリオ追加後
//#define LeftCourceOkiharaMode // 左コース用プログラム沖原コード
//#define RightCourceOkiharaMode // 右コース用プログラム沖原コード
//#define SlalomUFOTestMode // スラロームをUFO走行するプログラム。
//#define SlalomAwaitingSignalModePattern1_1 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1。案1
//#define SlalomAwaitingSignalModePattern2_1 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2。案1
//#define SlalomAwaitingSignalModePattern1_2 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1。案2
//#define SlalomAwaitingSignalModePattern2_2 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2。案2
//#define SlalomAwaitingSignalModePattern1_3 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1。案3
//#define SlalomAwaitingSignalModePattern2_3 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2。案3
//#define BlockTestMode  // ブロック搬入だけを走行するプログラム。
//#define FlatLineMode // すべて同じPIDで倉庫する左コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode // RGBRawの値をはかるプログラム
//#define ColorIDReaderMode // ColorIDを取得し続けるプログラム
//#define BrightnessReaderMode // 明るさを取得し続けるプログラム
//#define Rotate360TestMode // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
//#define RotateTestMode // 旋回モード。テスト用
//#define RotateGyroTestMode // ジャイロを使った旋回モード。テスト用。
//#define StraightTestMode // 直進モード。テスト用
//#define CurvatureWalkerTestMode // 曲率旋回モード。テスト用
//#define SwingSonarDetectorTestMode // 障害物距離角度首振り検出モード。テスト用
//#define ShigekiTestMode // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
//#define UFORunnerSwingTestMode // UFO走行モード。障害物間を向いている状態から始める。テスト用
//#define UFORunnerClockwiseTestMode // UFO走行モード。左障害物を向いている状態から始める。テスト用
//#define ColorPIDTracerTestMode // ColorPIDTraceを試すモード。テスト用
//#define BrightnessPIDTracerTestMode // TargetBrightnessPIDTraceを試すモード。テスト用
//#define FroggySongTestMode // かえるの歌を歌わせるモード。テスト用。
//#define GrayPredicateTestMode // グレーでとまる直進モード。テスト用。
//#define FacingAngleTestMode // 指定角度に向き直るモード。テスト用。
//#define WalkerTestMode
//#define BatteryEaaterMode // その場旋回をして電池を消費するモード。テスト用

// モード設定ここまで

// スラロームパターンの設定ここから。いずれか1つを有効化して
#define SlalomPattern1
//#define SlalomPattern2
// スラロームパターンの設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ
//#define SingASong       // 走行時に歌う
//#define EnablePrintGyroValue
//#define EnablePrintAngleUseWheel
//#define EnablePrintMotorCount
//#define EnablePrintPIDValues
//#define EnableRunnerTaskTimeCheck
#define DisableCalibration
#define StopWhenThrowException

// 使う走行体の選択ここから。キャリブレーションせずに固定値を使うときのrgb値決定に使われる（カラーセンサの個体差がすごい）
//#define RobotOkihara
//#define RobotKomichi
#define RobotSanekata
// 使う走行体の選択ここまで。

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
extern int angleFor360TurnMeasAngle;
extern DEBUG_LEVEL debugMessageLevel;
extern bool enablePrintMessageMode;
extern bool enablePrintMessageForLCD;
extern bool enablePrintMessageForConsole;
extern bool enablePrintMessageForBluetooth;
extern bool enableBeepWhenCommandSwitching;
extern Note *beepNoteWhenCommandSwitching;
extern int loopSong;
extern vector<Note *> song;

extern bool calibrateBlue;
extern bool calibrateBlueEdge;
extern bool calibrateSlalomWhite;
extern bool calibrateBlack;
extern bool calibrateWhite;
extern bool calibrateBlackWhiteEdge;
extern bool calibrateGray;

extern int blackWhiteEdgeTargetBrightness;

extern bool calibrateBlue;
extern bool calibrateBlueWhiteEdge;
extern bool calibrateWhiteAtSlalom;
extern bool calibrateBlack;
extern bool calibrateWhite;
extern bool calibrateBlackWhiteEdge;

extern int r_r;
extern int r_g;
extern int r_b;
extern RawColorPredicateCondition r_rCondition;
extern RawColorPredicateCondition r_gCondition;
extern RawColorPredicateCondition r_bCondition;
extern int g_r;
extern int g_g;
extern int g_b;
extern RawColorPredicateCondition g_rCondition;
extern RawColorPredicateCondition g_gCondition;
extern RawColorPredicateCondition g_bCondition;
extern int b_r;
extern int b_g;
extern int b_b;
extern RawColorPredicateCondition b_rCondition;
extern RawColorPredicateCondition b_gCondition;
extern RawColorPredicateCondition b_bCondition;
extern int y_r;
extern int y_g;
extern int y_b;
extern RawColorPredicateCondition y_rCondition;
extern RawColorPredicateCondition y_gCondition;
extern RawColorPredicateCondition y_bCondition;
extern int bw_r;
extern int bw_g;
extern int bw_b;
extern RawColorPredicateCondition bw_rCondition;
extern RawColorPredicateCondition bw_gCondition;
extern RawColorPredicateCondition bw_bCondition;
extern int blackR;
extern int black_g;
extern int d_b;
extern RawColorPredicateCondition d_rCondition;
extern RawColorPredicateCondition d_gCondition;
extern RawColorPredicateCondition d_bCondition;
extern int w_r;
extern int w_g;
extern int w_b;
extern RawColorPredicateCondition bw_rCondition;
extern RawColorPredicateCondition w_gCondition;
extern RawColorPredicateCondition w_bCondition;
extern int dw_r;
extern int dw_g;
extern int dw_b;
extern RawColorPredicateCondition dw_rCondition;
extern RawColorPredicateCondition dw_gCondition;
extern RawColorPredicateCondition dw_bCondition;
extern int sw_r;
extern int sw_g;
extern int sw_b;
extern RawColorPredicateCondition sw_rCondition;
extern RawColorPredicateCondition sw_gCondition;
extern RawColorPredicateCondition sw_bCondition;
extern int whiteR;
extern int whiteG;
extern int whiteB;
extern RawColorPredicateCondition WhiteRCondition;
extern RawColorPredicateCondition whiteGCondition;
extern RawColorPredicateCondition whiteBCondition;
extern int blackWhiteEdgeR;
extern int blackWhiteEdgeG;
extern int blackWhiteEdgeB;
extern RawColorPredicateCondition blackWhiteEdgeRCondition;
extern RawColorPredicateCondition blackWhiteEdgeGCondition;
extern RawColorPredicateCondition blackWhiteEdgeBCondition;
extern int whiteAtSlalomR;
extern int whiteAtSlalomG;
extern int whiteAtSlalomB;
extern RawColorPredicateCondition whiteAtSlalomRCondition;
extern RawColorPredicateCondition whiteAtSlalomGCondition;
extern RawColorPredicateCondition whiteAtSlalomBCondition;
extern int blackR;
extern int blackG;
extern int blackB;
extern RawColorPredicateCondition blackRCondition;
extern RawColorPredicateCondition blackGCondition;
extern RawColorPredicateCondition blackBCondition;
extern int redR;
extern int redG;
extern int redB;
extern RawColorPredicateCondition redRCondition;
extern RawColorPredicateCondition redGCondition;
extern RawColorPredicateCondition redBCondition;
extern int greenR;
extern int greenG;
extern int greenB;
extern RawColorPredicateCondition greenRCondition;
extern RawColorPredicateCondition greenGCondition;
extern RawColorPredicateCondition greenBCondition;
extern int blueR;
extern int blueG;
extern int blueB;
extern RawColorPredicateCondition blueRCondition;
extern RawColorPredicateCondition blueGCondition;
extern RawColorPredicateCondition blueBCondition;
extern int yellowR;
extern int yellowG;
extern int yellowB;
extern RawColorPredicateCondition yellowRCondition;
extern RawColorPredicateCondition yellowGCondition;
extern RawColorPredicateCondition yellowBCondition;
extern int grayR;
extern int grayG;
extern int grayB;
extern RawColorPredicateCondition grayRCondition;
extern RawColorPredicateCondition grayGCondition;
extern RawColorPredicateCondition grayBCondition;
extern int blueWhiteEdgeR;
extern int blueWhiteEdgeG;
extern int blueWhiteEdgeB;
extern RawColorPredicateCondition blueWhiteEdgeRCondition;
extern RawColorPredicateCondition blueWhiteEdgeGCondition;
extern RawColorPredicateCondition blueWhiteEdgeBCondition;

#endif