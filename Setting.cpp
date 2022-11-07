#include "Setting.h"
#include "ev3api.h"
#include "Note.h"
#include "RawColorPredicate.h"
#include "vector"
#include "MusicalScore.h"

using namespace std;

// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
// ********** 設定2/2ここから **********

// 車体情報設定ここから
#ifdef SimulatorMode // コメントアウトしないで

// シミュレータの車体情報設定ここから

float wheelSpace = 12;                      // 左車輪と右車輪の間隔。シミュレータ用
float distanceFromSonarSensorToAxle = 10.5; // ソナーセンサから車軸までの距離。シミュレータ用
float wheelDiameter = 10.4;                 // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;   // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 520;  // 右に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnMeasAngle = 1065;        // 片方の車輪で360度旋回するために必要な回転角
int pwmForResetArm = 10;                    // ResetArmAngleで使われるPWM
int numberOfTimesForPullWhenResetArm = 25;  // ResetArmAngleの初回動作、ー無を引っ張るフレーム数
int angleForResetArm = 30;                  // ResetArmAngleによって設定されるアーム角度
float leftWheelPWMCorrectedValue = 1;       // 右モータPWM補正値
float rightWheelPWMCorrectedValue = 1;      // 左モータPWM補正値
float armMotorPWMCorrectedValue = 1;        // アームモータPWM補正値
float tailMotorPWMCorrectedValue = 1;       // テールモータPWM補正値

// シミュレータの車体情報設定ここまで

#else

// 実機の車体情報設定ここから

float wheelSpace = 14.5;                   // 左車輪と右車輪の間隔。実機用
float distanceFromSonarSensorToAxle = 11;  // ソナーセンサから車軸までの距離。実機用
float wheelDiameter = 10.4;                // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;  // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnMeasAngle = 1120;       // 片方の車輪で360度旋回するために必要な回転角
int pwmForResetArm = 10;                   // ResetArmAngleで使われるPWM
int numberOfTimesForPullWhenResetArm = 25; // ResetArmAngleの初回動作、ー無を引っ張るフレーム数
int angleForResetArm = 30;                 // ResetArmAngleによって設定されるアーム角度
float leftWheelPWMCorrectedValue = 1;      // 右モータPWM補正値
float rightWheelPWMCorrectedValue = 1;     // 左モータPWM補正値
float armMotorPWMCorrectedValue = 1;       // アームモータPWM補正値
float tailMotorPWMCorrectedValue = 1;      // テールモータPWM補正値

// 実機の車体情報設定ここまで

#endif

// 車体情報設定ここまで

// 情報出力の有効無効設定ここから

DEBUG_LEVEL debugMessageLevel = NONE;        // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = false;         // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForLCD = false;       // trueにすると、本体画面に情報がprintされる。（enablePrintMessageMode をtrueにする必要がある）
bool enablePrintMessageForConsole = false;   // trueにすると、コンソールにも情報がprintされる。（enablePrintMessageMode をtrueにする必要がある）
bool enablePrintMessageForBluetooth = false; // trueにすると、Bluetooth接続端末にも情報がprintされる。（enablePrintMessageModeをtrueにし、ヘッダファイルの#define EnableBluetoothのコメントアウトを外す必要がある）
int lowBatteryVoltageMv = 7650;

// 情報出力の有効無効設定ここまで

// コマンド切り替え時ビープ音LED設定ここから

bool enableBeepWhenCommandSwitching = false; // trueにすると、コマンド切り替え時にビープ音を鳴らす。
bool enableBeepWhenCommandSwitchingSong = false;
Note *beepNoteWhenCommandSwitching = new Note(NOTE_C4, 50, 5); // debugBeep()で鳴らす音の定義
vector<Note *> song = generateFroggySong();                    // 流す曲の定義
int loopSong = 10;                                             // 曲のループ数

bool enableSwitchLEDWhenCommandSwitching = true; // trueにすると、コマンド切り替え時にLEDの色を切り替える。
ledcolor_t ledColorsWhenCommandSwitchingArray[] = {
    LED_RED,
    LED_GREEN,
    LED_ORANGE,
}; // debugLEDで切り替えるときのLEDの色定義
vector<ledcolor_t> ledColorsWhenCommandSwitching(ledColorsWhenCommandSwitchingArray, ledColorsWhenCommandSwitchingArray + sizeof(ledColorsWhenCommandSwitchingArray) / sizeof(ledColorsWhenCommandSwitchingArray[0]));

// コマンド切り替え時ビープ音LED設定ここまで

// キャリブレーション設定ここから

char preCalibratedValuesFileName[] = "/PreCalibratedValues.ini"; // キャリブレーションした値を保存するファイルのpath

// 各色をキャリブレーションするかどうかの設定ここから

bool calibrateWhiteAtSlalom = true;
bool calibrateGray = true;
bool calibrateBlack = true;
bool calibrateWhite = true;
bool calibrateBlackWhiteEdge = true;
bool calibrateBlueWhiteEdge = true;
bool calibrateRed = true;
bool calibrateGreen = true;
bool calibrateBlue = true;
bool calibrateYellow = true;
bool calibrateRedCard = true;
bool calibrateGreenCard = true;
bool calibrateBlueCard = true;
bool calibrateYellowCard = true;

// 各色をキャリブレーションするかどうかの設定ここまで

// 以下色の値設定
// 設定しなくてOK。キャリブレーター使えば上書きされますし、
// キャリブレーション完了してから上ボタンで値を保存、キャリブレーション中に下ボタンで保存した値を読み込みできます。

#ifdef SimulatorMode
int blackWhiteEdgeTargetBrightness = 20;
int whiteBrightness = 0;
int blackBrightness = 0;
#else
int blackWhiteEdgeTargetBrightness = (30 + 2) / 2;
int whiteBrightness = 30;
int blackBrightness = 2;
#endif

// スラローム上からみた白（試走会では値取れていないので学校の値）
#ifdef SimulatorMode
int whiteAtSlalomR = 0;
int whiteAtSlalomG = 0;
int whiteAtSlalomB = 0;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN15;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN10;
#else
int whiteAtSlalomR = 61;
int whiteAtSlalomG = 59;
int whiteAtSlalomB = 84;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN10;
#endif

// グレー
int grayR = 33;
int grayG = 40;
int grayB = 50;
RawColorPredicateCondition grayRCondition = BETWEEN5;
RawColorPredicateCondition grayGCondition = BETWEEN5;
RawColorPredicateCondition grayBCondition = BETWEEN5;

// 黒（試走会では値取れていないので学校の値）
int blackR = 8;
int blackG = 8;
int blackB = 10;
RawColorPredicateCondition blackRCondition = BETWEEN5;
RawColorPredicateCondition blackGCondition = BETWEEN5;
RawColorPredicateCondition blackBCondition = BETWEEN5;

// 白（試走会では値取れていないので学校の値）
int whiteR = 72;
int whiteG = 72;
int whiteB = 94;
RawColorPredicateCondition WhiteRCondition = BETWEEN5;
RawColorPredicateCondition whiteGCondition = BETWEEN5;
RawColorPredicateCondition whiteBCondition = BETWEEN5;

// 黒白境界（試走会では値取れていないので学校の値）
int blackWhiteEdgeR = 38;
int blackWhiteEdgeG = 42;
int blackWhiteEdgeB = 51;
RawColorPredicateCondition blackWhiteEdgeRCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeGCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeBCondition = BETWEEN5;

// 青白境界（試走会では値取れていないので学校の値）
int blueWhiteEdgeR = 36;
int blueWhiteEdgeG = 56;
int blueWhiteEdgeB = 92;
RawColorPredicateCondition blueWhiteEdgeRCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeGCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeBCondition = BETWEEN5;

// 赤
int redR = 70;
int redG = 23;
int redB = 20;
RawColorPredicateCondition redRCondition = BETWEEN10;
RawColorPredicateCondition redGCondition = BETWEEN10;
RawColorPredicateCondition redBCondition = BETWEEN5;

// 緑
int greenR = 12;
int greenG = 45;
int greenB = 21;
RawColorPredicateCondition greenRCondition = BETWEEN10;
RawColorPredicateCondition greenGCondition = BETWEEN10;
RawColorPredicateCondition greenBCondition = BETWEEN10;

// 青
int blueR = 4;
int blueG = 21;
int blueB = 54;
RawColorPredicateCondition blueRCondition = BETWEEN10;
RawColorPredicateCondition blueGCondition = BETWEEN10;
RawColorPredicateCondition blueBCondition = BETWEEN10;

// 黄
int yellowR = 75;
int yellowG = 66;
int yellowB = 16;
RawColorPredicateCondition yellowRCondition = BETWEEN10;
RawColorPredicateCondition yellowGCondition = BETWEEN10;
RawColorPredicateCondition yellowBCondition = BETWEEN10;

// 赤カード
int redCardR = 70;
int redCardG = 23;
int redCardB = 20;
RawColorPredicateCondition redCardRCondition = BETWEEN10;
RawColorPredicateCondition redCardGCondition = BETWEEN10;
RawColorPredicateCondition redCardBCondition = BETWEEN5;

// 緑カード
int greenCardR = 12;
int greenCardG = 45;
int greenCardB = 21;
RawColorPredicateCondition greenCardRCondition = BETWEEN10;
RawColorPredicateCondition greenCardGCondition = BETWEEN10;
RawColorPredicateCondition greenCardBCondition = BETWEEN10;

// 青カード
int blueCardR = 4;
int blueCardG = 21;
int blueCardB = 54;
RawColorPredicateCondition blueCardRCondition = BETWEEN10;
RawColorPredicateCondition blueCardGCondition = BETWEEN10;
RawColorPredicateCondition blueCardBCondition = BETWEEN10;

// 黄カード
int yellowCardR = 75;
int yellowCardG = 66;
int yellowCardB = 16;
RawColorPredicateCondition yellowCardRCondition = BETWEEN10;
RawColorPredicateCondition yellowCardGCondition = BETWEEN10;
RawColorPredicateCondition yellowCardBCondition = BETWEEN10;

// キャリブレーション設定ここまで

// ********** 設定2/2ここまで **********
