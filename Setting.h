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
//#define SanekataCanNotGoal // ゴール以外の結合がしたい実方用。コメントアウトして。

// コース設定ここから。いずれか1つを有効化して
#define Left
//#define Right
// コース設定ここまで

// スラロームパターンの設定ここから。いずれか1つを有効化して
//#define SlalomPattern1
#define SlalomPattern2
// スラロームパターンの設定ここまで

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
//#define TrueCourceOkiharaModeCS // 完走用プログラム。沖原トレース
//#define TrueCourceKomichiModeCS // 完走用プログラム。小路シナリオ
//#define TrueCourceOkiharaModeRegional // 完走用プログラム。沖原トレース
//#define TrueCourceKomichiModeRegional  // 完走用プログラム。小路シナリオ
//#define GoalMode // ゴール用プログラム。試走会1時点。
//#define GoalOkiharaMode // 左コース用プログラム。沖原作業用
//#define GoalOkiharaMode1 // 左コース用プログラム。地区大会時点安定
//#define GoalOkiharaMode2 // 左コース用プログラム
//#define GoalOkiharaMode3 // 左コース用プログラム。1のスピードアップ版2022-10-18
//#define GoalOkiharaMode3Distance
//#define KomichiScnenarioTestMode // 完全シナリオ小路コード
//#define SanekataScenarioTestMode // 部分シナリオ実方コード
//#define SlalomAwaitingSignalPlan1TestMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案1
//#define SlalomAwaitingSignalPlan2TestMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案2
//#define SlalomAwaitingSignalPlan3TestMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案3
//#define SlalomAwaitingSignalPlan4TestMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案3
#define SlalomAwaitingSignalPlan5TestMode // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案3
//#define BlockTestMode // ブロック搬入だけを走行するプログラム。小路作業用
//#define FlatLineMode // すべて同じPIDで走行する左コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム。試走会用
//#define RGBRawReaderMode // RGBRawの値をはかるプログラム。試走会用
//#define ColorIDReaderMode // ColorIDを取得し続けるプログラム。試走会用
//#define ColorReaderUseRawTestMode // RawColorで色を取得し続けるプログラム。試走会用。
//#define BrightnessReaderMode // 明るさを取得し続けるプログラム。試走会用
//#define Rotate360TestMode // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
//#define RotateTestMode // 旋回モード。テスト用
//#define RotateGyroTestMode // ジャイロを使った旋回モード。テスト用。
//#define StraightTestMode // 直進モード。テスト用
//#define CurvatureWalkerTestMode // 曲率旋回モード。テスト用
//#define SwingSonarDetectorTestMode // 障害物距離角度首振り検出モード。テスト用
//#define ShigekiTestMode // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
//#define SlalomUFOTestMode // スラロームだけをUFO走行するプログラム。
//#define UFORunnerSwingTestMode // UFO走行モード。障害物間を向いている状態から始める。テスト用
//#define UFORunnerClockwiseTestMode // UFO走行モード。左障害物を向いている状態から始める。テスト用
//#define ColorPIDTracerTestMode // ColorPIDTraceを試すモード。テスト用
//#define ColorPIDTracerV2TestMode // ColorPIDTraceV2を試すモード。テスト用
//#define BrightnessPIDTracerTestMode // TargetBrightnessPIDTraceを試すモード。テスト用
//#define BrightnessPIDTracerV2TestMode // ColorPIDTraceV2を試すモード。テスト用
//#define FroggySongTestMode // かえるの歌を歌わせるモード。テスト用。
//#define GrayPredicateTestMode // グレーでとまる直進モード。テスト用。
//#define FacingAngleTestMode // 指定角度に向き直るモード。テスト用。
//#define WalkerTestMode // Walkerで走るモード。テスト用。
//#define BatteryEaaterMode // その場旋回をして電池を消費するモード。テスト用
//#define SlalomBlockJoinTestMode // スラロームとブロックの結合を試すモード。テスト用。
//#define PIDStraightWalkerTestMode
//#define PIDFacingAngleAbsTestMode
//#define KomichiBlockTestMode

// モード設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ
//#define SingASong       // 走行時に歌う
//#define EnablePrintGyroValue // コメントアウトを外すとジャイロの値を出力する
//#define EnablePrintAngleUseWheel // コメントアウトを外すと車輪回転角から導き出された車体旋回角度を出力する
//#define EnablePrintMotorCount // コメントアウトを外すとモータの回転数を出力する
//#define EnablePrintSonarDistance
//#define EnablePrintPIDValues // コメントアウトを外すと各PIDTracerのPID値などを出力する
//#define EnableRunnerTaskTimeCheck // コメントアウトを外すと、RunnerTask開始と終了のタイミングを出力する
//#define EnablePrintCommandName // コマンド切り替え時にコマンド名を出力する。キャリブレータの表示より優先されるっぽいのでデバッグ目的以外ではオフっといたほうがいいかも
#define StopWhenThrowException // コメントアウトを外すと、例外が飛んだときにデデドンして止まる

// ********** 設定1/2ここまで **********

enum DEBUG_LEVEL
{
    NONE,
    INFO,
    DEBUG,
    TRACE,
};

extern char preCalibratedValuesFileName[];

extern float wheelDiameter;
extern float distanceFromSonarSensorToAxle;
extern float wheelSpace;
extern int angleFor360TurnLeftRotateRobot;
extern int angleFor360TurnRightRotateRobot;
extern int angleFor360TurnMeasAngle;
extern int pwmForResetArm;
extern int angleForResetArm;
extern int numberOfTimesForPullWhenResetArm;

extern float leftWheelPWMCorrectedValue;
extern float rightWheelPWMCorrectedValue;
extern float armMotorPWMCorrectedValue;
extern float tailMotorPWMCorrectedValue;

extern DEBUG_LEVEL debugMessageLevel;
extern bool enablePrintMessageMode;
extern bool enablePrintMessageForLCD;
extern bool enablePrintMessageForConsole;
extern bool enablePrintMessageForBluetooth;
extern bool enableBeepWhenCommandSwitching;
extern bool enableBeepWhenCommandSwitchingSong;
extern Note *beepNoteWhenCommandSwitching;
extern bool enableSwitchLEDWhenCommandSwitching;
extern vector<ledcolor_t> ledColorsWhenCommandSwitching;
extern int loopSong;
extern vector<Note *> song;

extern bool calibrateRed;
extern bool calibrateGreen;
extern bool calibrateBlue;
extern bool calibrateYellow;

extern bool calibrateRedCard;
extern bool calibrateGreenCard;
extern bool calibrateBlueCard;
extern bool calibrateYellowCard;

extern bool calibrateBlueEdge;
extern bool calibrateSlalomWhite;
extern bool calibrateBlack;
extern bool calibrateWhite;
extern bool calibrateBlackWhiteEdge;
extern bool calibrateGray;
extern bool calibrateBlueWhiteEdge;
extern bool calibrateWhiteAtSlalom;
extern bool calibrateBlack;
extern bool calibrateWhite;
extern bool calibrateBlackWhiteEdge;

extern int blackWhiteEdgeTargetBrightness;
extern int whiteBrightness;
extern int blackBrightness;
extern int whiteAtSlalomR;
extern int whiteAtSlalomG;
extern int whiteAtSlalomB;
extern RawColorPredicateCondition whiteAtSlalomRCondition;
extern RawColorPredicateCondition whiteAtSlalomGCondition;
extern RawColorPredicateCondition whiteAtSlalomBCondition;
extern int grayR;
extern int grayG;
extern int grayB;
extern RawColorPredicateCondition grayRCondition;
extern RawColorPredicateCondition grayGCondition;
extern RawColorPredicateCondition grayBCondition;
extern int blackR;
extern int blackG;
extern int blackB;
extern RawColorPredicateCondition blackRCondition;
extern RawColorPredicateCondition blackGCondition;
extern RawColorPredicateCondition blackBCondition;
extern int whiteR;
extern int whiteG;
extern int whiteB;
extern RawColorPredicateCondition whiteRCondition;
extern RawColorPredicateCondition whiteGCondition;
extern RawColorPredicateCondition whiteBCondition;
extern int blackWhiteEdgeR;
extern int blackWhiteEdgeG;
extern int blackWhiteEdgeB;
extern RawColorPredicateCondition blackWhiteEdgeRCondition;
extern RawColorPredicateCondition blackWhiteEdgeGCondition;
extern RawColorPredicateCondition blackWhiteEdgeBCondition;
extern int blueWhiteEdgeR;
extern int blueWhiteEdgeG;
extern int blueWhiteEdgeB;
extern RawColorPredicateCondition blueWhiteEdgeRCondition;
extern RawColorPredicateCondition blueWhiteEdgeGCondition;
extern RawColorPredicateCondition blueWhiteEdgeBCondition;
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

extern int redCardR;
extern int redCardG;
extern int redCardB;
extern RawColorPredicateCondition redCardRCondition;
extern RawColorPredicateCondition redCardGCondition;
extern RawColorPredicateCondition redCardBCondition;
extern int greenCardR;
extern int greenCardG;
extern int greenCardB;
extern RawColorPredicateCondition greenCardRCondition;
extern RawColorPredicateCondition greenCardGCondition;
extern RawColorPredicateCondition greenCardBCondition;
extern int blueCardR;
extern int blueCardG;
extern int blueCardB;
extern RawColorPredicateCondition blueCardRCondition;
extern RawColorPredicateCondition blueCardGCondition;
extern RawColorPredicateCondition blueCardBCondition;
extern int yellowCardR;
extern int yellowCardG;
extern int yellowCardB;
extern RawColorPredicateCondition yellowCardRCondition;
extern RawColorPredicateCondition yellowCardGCondition;
extern RawColorPredicateCondition yellowCardBCondition;

#endif