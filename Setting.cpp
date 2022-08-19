#include "Setting.h"
#include "ev3api.h"

// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// ********** 設定2/2ここから **********

// 車体情報設定ここから

float wheelDiameter = 10.4;                 // 車輪直径。センチメートル。
float distanceFromSonarSensorToAxle = 10.5; // ソナーセンサから車軸までの距離
float wheelSpace = 14.5;                    // 左車輪と右車輪の間隔
int angleFor360TurnLeftRotateRobot = 520;   // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510;  // 右に360度旋回するのに必要な左右車輪回転角度数

// 車体情報設定ここまで

// 情報出力の有効無効設定ここから

DEBUG_LEVEL debugMessageLevel = TRACE;      // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = true;         // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForConsole = true;   // trueにすると、コンソールにも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）
bool enablePrintMessageForBluetooth = true; // trueにすると、Bluetooth接続端末にも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）trueにする場合、ヘッダファイルの#define EnableBluetoothのコメントアウトも外して。

// 情報出力の有効無効設定ここまで

// ********** 設定2/2ここまで **********

// TODO angleFor360の左右対応が逆になってるっぽいな