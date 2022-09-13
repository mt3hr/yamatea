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

// シミュレータの車体情報設定ここまで

#else

// 実機の車体情報設定ここから

float wheelSpace = 14.5;                   // 左車輪と右車輪の間隔。実機用
float distanceFromSonarSensorToAxle = 11;  // ソナーセンサから車軸までの距離。実機用
float wheelDiameter = 10.4;                // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;  // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnMeasAngle = 1120;       // 片方の車輪で360度旋回するために必要な回転角

// 実機の車体情報設定ここまで

#endif

// 車体情報設定ここまで

// 情報出力の有効無効設定ここから

DEBUG_LEVEL debugMessageLevel = NONE;        // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = false;         // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForLCD = false;       // trueにすると、本体画面に情報がprintされる。（enablePrintMessageMode をtrueにする必要がある）
bool enablePrintMessageForConsole = false;   // trueにすると、コンソールにも情報がprintされる。（enablePrintMessageMode をtrueにする必要がある）
bool enablePrintMessageForBluetooth = false; // trueにすると、Bluetooth接続端末にも情報がprintされる。（enablePrintMessageModeをtrueにし、ヘッダファイルの#define EnableBluetoothのコメントアウトを外す必要がある）

// 情報出力の有効無効設定ここまで

// コマンド切り替え時ビープ音設定ここから

bool enableBeepWhenCommandSwitching = true; // trueにすると、コマンド切り替え時にビープ音を鳴らす。
Note *beepNoteWhenCommandSwitching = new Note(NOTE_C6, 50, 30);
vector<Note *> song = generateFroggySong();
int loopSong = 10;

// コマンド切り替え時ビープ音設定ここまで

// 色設定ここから

bool calibrateBlue = false;         // 青色をキャリブレーションするかどうか
bool calibrateBlueWhiteEdge = true; // 青白エッジをキャリブレーションするかどうか
bool calibrateWhiteAtSlalom = true; // スラローム上からみた白をキャリブレーションするかどうか
bool calibrateBlack = true;
bool calibrateWhite = true;
bool calibrateGray = true;
bool calibrateBlackWhiteEdge = true;

#ifdef SimulatorMode
int blackWhiteEdgeTargetBrightness = 20;
int whiteBrightness = 0;
int blackBrightness = 0;
#else
int blackWhiteEdgeTargetBrightness = 0;
int whiteBrightness = 0;
int blackBrightness = 0;
#endif

char preCalibratedValuesFileName[] = "/PreCalibratedValues.ini";

// 白（キャリブレータから上書きされるので設定しなくて良い）
int whiteR = 0;
int whiteG = 0;
int whiteB = 0;
RawColorPredicateCondition WhiteRCondition = BETWEEN5;
RawColorPredicateCondition whiteGCondition = BETWEEN5;
RawColorPredicateCondition whiteBCondition = BETWEEN5;

// 黒白境界（キャリブレータから上書きされるので設定しなくて良い）
int blackWhiteEdgeR = 0;
int blackWhiteEdgeG = 0;
int blackWhiteEdgeB = 0;
RawColorPredicateCondition blackWhiteEdgeRCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeGCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeBCondition = BETWEEN5;

// スラローム上からみた白（キャリブレータから上書きされるので設定しなくて良い）
#ifdef SimulatorMode
int whiteAtSlalomR = 0;
int whiteAtSlalomG = 0;
int whiteAtSlalomB = 0;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN5;
#else
int whiteAtSlalomR = 0;
int whiteAtSlalomG = 0;
int whiteAtSlalomB = 0;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN10;
#endif

// 黒（キャリブレータから上書きされるので設定しなくて良い）
int blackR = 0;
int blackG = 0;
int blackB = 0;
RawColorPredicateCondition blackRCondition = BETWEEN5;
RawColorPredicateCondition blackGCondition = BETWEEN5;
RawColorPredicateCondition blackBCondition = BETWEEN5;

// 赤
int redR = 0;
int redG = 0;
int redB = 0;
RawColorPredicateCondition redRCondition = BETWEEN5;
RawColorPredicateCondition redGCondition = BETWEEN5;
RawColorPredicateCondition redBCondition = BETWEEN5;

// 緑
int greenR = 0;
int greenG = 0;
int greenB = 0;
RawColorPredicateCondition greenRCondition = BETWEEN5;
RawColorPredicateCondition greenGCondition = BETWEEN5;
RawColorPredicateCondition greenBCondition = BETWEEN5;

// 青（キャリブレータから上書きされるので設定しなくて良い）
int blueR = 0;
int blueG = 0;
int blueB = 0;
RawColorPredicateCondition blueRCondition = BETWEEN3;
RawColorPredicateCondition blueGCondition = BETWEEN3;
RawColorPredicateCondition blueBCondition = BETWEEN3;

// 黄
int yellowR = 0;
int yellowG = 0;
int yellowB = 0;
RawColorPredicateCondition yellowRCondition = BETWEEN5;
RawColorPredicateCondition yellowGCondition = BETWEEN5;
RawColorPredicateCondition yellowBCondition = BETWEEN5;

// グレー（キャリブレータから上書きされるので設定しなくて良い）
int grayR = 0;
int grayG = 0;
int grayB = 0;
RawColorPredicateCondition grayRCondition = BETWEEN5;
RawColorPredicateCondition grayGCondition = BETWEEN5;
RawColorPredicateCondition grayBCondition = BETWEEN5;

// 青白境界（キャリブレータから上書きされるので設定しなくて良い）//TODO エッジを実測して。（平均を取るのではダメらしい）
int blueWhiteEdgeR = 0;
int blueWhiteEdgeG = 0;
int blueWhiteEdgeB = 0;
RawColorPredicateCondition blueWhiteEdgeRCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeGCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeBCondition = BETWEEN10;
// ********** 設定2/2ここまで **********
