#include "Setting.h"
#include "ev3api.h"

// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
// ********** 設定2/2ここから **********

// 車体情報設定ここから
#ifdef SimulatorMode
float wheelSpace = 12;                                                // 左車輪と右車輪の間隔。シミュレータ用
float distanceFromSonarSensorToAxle = 10.5;                           // ソナーセンサから車軸までの距離。シミュレータ用
float wheelDiameter = 10.4;                                           // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;                             // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = angleFor360TurnLeftRotateRobot; // 右に360度旋回するのに必要な左右車輪回転角度数
#else
float wheelSpace = 14.5;                   // 左車輪と右車輪の間隔。実機用
float distanceFromSonarSensorToAxle = 11;  // ソナーセンサから車軸までの距離。実機用
float wheelDiameter = 10.4;                // 車輪直径。センチメートル。
int angleFor360TurnLeftRotateRobot = 520;  // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数
#endif

// 車体情報設定ここまで

// 情報出力の有効無効設定ここから

DEBUG_LEVEL debugMessageLevel = NONE;        // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = false;         // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForConsole = false;   // trueにすると、コンソールにも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）
bool enablePrintMessageForBluetooth = false; // trueにすると、Bluetooth接続端末にも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）trueにする場合、ヘッダファイルの#define EnableBluetoothのコメントアウトも外して。
bool enableBeepWhenCommandSwitching = false; // trueにすると、コマンド切り替え時にビープ音を鳴らす。

// 情報出力の有効無効設定ここまで

// ********** 設定2/2ここまで **********
