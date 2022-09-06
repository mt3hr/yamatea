#include "Setting.h"
#include "ev3api.h"
#include "Note.h"
#include "ev3api.h"
#include "RawColorPredicate.h"

// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
// ********** 設定2/2ここから **********

// 車体情報設定ここから
#ifdef SimulatorMode

// シミュレータの車体情報設定ここから

float wheelSpace = 12;                                                // 左車輪と右車輪の間隔。シミュレータ用
float distanceFromSonarSensorToAxle = 10.5;                           // ソナーセンサから車軸までの距離。シミュレータ用
float wheelDiameter = 10.4;                                           // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;                             // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = angleFor360TurnLeftRotateRobot; // 右に360度旋回するのに必要な左右車輪回転角度数

// シミュレータの車体情報設定ここまで

#else

// 実機の車体情報設定ここから

float wheelSpace = 14.5;                   // 左車輪と右車輪の間隔。実機用
float distanceFromSonarSensorToAxle = 11;  // ソナーセンサから車軸までの距離。実機用
float wheelDiameter = 10.4;                // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;  // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数

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
Note *beepNoteWhenCommandSwitching = new Note(NOTE_C4, 50, 30);
int loopSong = 10;

// コマンド切り替え時ビープ音設定ここまで

// 色設定ここから

bool calibrateBlue = true; // 青色をキャリブレーションするかどうか

// 白（キャリブレータから上書きされるので設定しなくて良い）
int w_r = 70;
int w_g = 76;
int w_b = 55;
RawColorPredicateCondition w_rCondition = BETWEEN5;
RawColorPredicateCondition w_gCondition = BETWEEN5;
RawColorPredicateCondition w_bCondition = BETWEEN5;

// 黒（キャリブレータから上書きされるので設定しなくて良い）
int d_r = 6;
int d_g = 6;
int d_b = 5;
RawColorPredicateCondition d_rCondition = BETWEEN5;
RawColorPredicateCondition d_gCondition = BETWEEN5;
RawColorPredicateCondition d_bCondition = BETWEEN5;

// 赤
int r_r = 0;
int r_g = 0;
int r_b = 0;
RawColorPredicateCondition r_rCondition = BETWEEN5;
RawColorPredicateCondition r_gCondition = BETWEEN5;
RawColorPredicateCondition r_bCondition = BETWEEN5;

// 緑
int g_r = 0;
int g_g = 0;
int g_b = 0;
RawColorPredicateCondition g_rCondition = BETWEEN5;
RawColorPredicateCondition g_gCondition = BETWEEN5;
RawColorPredicateCondition g_bCondition = BETWEEN5;

// 青（キャリブレータから上書きされるので設定しなくて良い）
int b_r = 29;
int b_g = 47;
int b_b = 42;
RawColorPredicateCondition b_rCondition = BETWEEN5;
RawColorPredicateCondition b_gCondition = BETWEEN5;
RawColorPredicateCondition b_bCondition = BETWEEN5;

// 黄
int y_r = 0;
int y_g = 0;
int y_b = 0;
RawColorPredicateCondition y_rCondition = BETWEEN5;
RawColorPredicateCondition y_gCondition = BETWEEN5;
RawColorPredicateCondition y_bCondition = BETWEEN5;

// 青白境界（キャリブレータから上書きされるので設定しなくて良い）//TODO エッジを実測して。（平均を取るのではダメらしい）
int bw_r = (w_r + b_r) / 2;
int bw_g = (w_g + b_g) / 2;
int bw_b = (w_b + b_b) / 2;
RawColorPredicateCondition bw_rCondition = LESS_THAN;
RawColorPredicateCondition bw_gCondition = BETWEEN5;
RawColorPredicateCondition bw_bCondition = BETWEEN5;

// 色設定ここまで

// ********** 設定2/2ここまで **********
