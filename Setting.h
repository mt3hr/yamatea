#ifndef Setting_H
#define Setting_H

// app.cppから参照される設定変数の定義。
// includeするためにappから分離する必要があったため、このファイルを作成しました。
//
// 実方

extern bool enablePrintMessageMode;
extern bool enablePrintMessageForBluetooth;
extern bool enablePrintMessageForConsole;
extern float wheelDiameter;

extern int angleFor360TurnRightRotateRobot; // 左に360度旋回するのに必要な左右車輪回転角度数
extern int angleFor360TurnLeftRotateRobot;  // 右に360度旋回するのに必要な左右車輪回転角度数

extern float wheelSpace; // 右車輪と左車輪の間隔。

extern float distanceFromSonarSensorToAxle; // ソナーセンサから車軸までの距離

extern bool enablePrintDebugMessage; // DebugUtilの関数群の出力処理有効化無効化。

#endif