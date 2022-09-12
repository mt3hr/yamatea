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

DEBUG_LEVEL debugMessageLevel = NONE;      // TournamentTODO // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = false;         // TournamentTODO // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForLCD = false;      // TournamentTODO // trueにすると、本体画面に情報がprintされる。（enablePrintMessageMode をtrueにする必要がある）
bool enablePrintMessageForConsole = false;  // TournamentTODO // trueにすると、コンソールにも情報がprintされる。（enablePrintMessageMode をtrueにする必要がある）
bool enablePrintMessageForBluetooth = false; // TournamentTODO // trueにすると、Bluetooth接続端末にも情報がprintされる。（enablePrintMessageModeをtrueにし、ヘッダファイルの#define EnableBluetoothのコメントアウトを外す必要がある）

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
#else
int blackWhiteEdgeTargetBrightness = 12; // TournamentTODO
#endif

#ifdef RobotSanekata
// 白（キャリブレータから上書きされるので設定しなくて良い）
int whiteR = 69; // TournamentTODO
int whiteG = 66; // TournamentTODO
int whiteB = 81; // TournamentTODO
RawColorPredicateCondition WhiteRCondition = BETWEEN5;
RawColorPredicateCondition whiteGCondition = BETWEEN5;
RawColorPredicateCondition whiteBCondition = BETWEEN5;

// 黒白境界（キャリブレータから上書きされるので設定しなくて良い）
int blackWhiteEdgeR = 45; // TournamentTODO
int blackWhiteEdgeG = 52; // TournamentTODO
int blackWhiteEdgeB = 60; // TournamentTODO
RawColorPredicateCondition blackWhiteEdgeRCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeGCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeBCondition = BETWEEN5;

// スラローム上からみた白（キャリブレータから上書きされるので設定しなくて良い）
#ifdef SimulatorMode
int whiteAtSlalomR = 115;
int whiteAtSlalomG = 110;
int whiteAtSlalomB = 155;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN5;
#else
int whiteAtSlalomR = 64; // TournamentTODO
int whiteAtSlalomG = 61; // TournamentTODO
int whiteAtSlalomB = 83; // TournamentTODO
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN10;
#endif

// 黒（キャリブレータから上書きされるので設定しなくて良い）
int blackR = 6; // TournamentTODO
int blackG = 6; // TournamentTODO
int blackB = 6; // TournamentTODO
RawColorPredicateCondition blackRCondition = BETWEEN5;
RawColorPredicateCondition blackGCondition = BETWEEN5;
RawColorPredicateCondition blackBCondition = BETWEEN5;

// 赤
int redR = 68; // TournamentTODO
int redG = 23; // TournamentTODO
int redB = 19; // TournamentTODO
RawColorPredicateCondition redRCondition = BETWEEN5;
RawColorPredicateCondition redGCondition = BETWEEN5;
RawColorPredicateCondition redBCondition = BETWEEN5;

// 緑
int greenR = 9;  // TournamentTODO
int greenG = 33; // TournamentTODO
int greenB = 18; // TournamentTODO
RawColorPredicateCondition greenRCondition = BETWEEN5;
RawColorPredicateCondition greenGCondition = BETWEEN5;
RawColorPredicateCondition greenBCondition = BETWEEN5;

// 青（キャリブレータから上書きされるので設定しなくて良い）
int blueR = 6;  // TournamentTODO
int blueG = 22; // TournamentTODO
int blueB = 54; // TournamentTODO
RawColorPredicateCondition blueRCondition = BETWEEN3;
RawColorPredicateCondition blueGCondition = BETWEEN3;
RawColorPredicateCondition blueBCondition = BETWEEN3;

// 黄
int yellowR = 70; // TournamentTODO
int yellowG = 57; // TournamentTODO
int yellowB = 15; // TournamentTODO
RawColorPredicateCondition yellowRCondition = BETWEEN5;
RawColorPredicateCondition yellowGCondition = BETWEEN5;
RawColorPredicateCondition yellowBCondition = BETWEEN5;

// グレー（キャリブレータから上書きされるので設定しなくて良い）
int grayR = 29; // TournamentTODO
int grayG = 40; // TournamentTODO
int grayB = 47; // TournamentTODO
RawColorPredicateCondition grayRCondition = BETWEEN5;
RawColorPredicateCondition grayGCondition = BETWEEN5;
RawColorPredicateCondition grayBCondition = BETWEEN5;

// 青白境界（キャリブレータから上書きされるので設定しなくて良い）//TODO エッジを実測して。（平均を取るのではダメらしい）
int blueWhiteEdgeR = 31; // TournamentTODO
int blueWhiteEdgeG = 46; // TournamentTODO
int blueWhiteEdgeB = 73; // TournamentTODO
RawColorPredicateCondition blueWhiteEdgeRCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeGCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeBCondition = BETWEEN10;
#endif

#ifdef RobotOkihara
// 白（キャリブレータから上書きされるので設定しなくて良い）
int whiteR = 74; // TournamentTODO
int whiteG = 77; // TournamentTODO
int whiteB = 59; // TournamentTODO
RawColorPredicateCondition WhiteRCondition = BETWEEN5;
RawColorPredicateCondition whiteGCondition = BETWEEN5;
RawColorPredicateCondition whiteBCondition = BETWEEN5;

// 黒白境界（キャリブレータから上書きされるので設定しなくて良い）
int blackWhiteEdgeR = 32; // TournamentTODO
int blackWhiteEdgeG = 40; // TournamentTODO
int blackWhiteEdgeB = 26; // TournamentTODO
RawColorPredicateCondition blackWhiteEdgeRCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeGCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeBCondition = BETWEEN5;

// スラローム上からみた白（キャリブレータから上書きされるので設定しなくて良い）
#ifdef SimulatorMode
int whiteAtSlalomR = 115;
int whiteAtSlalomG = 110;
int whiteAtSlalomB = 155;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN5;
#else
int whiteAtSlalomR = 62; // TournamentTODO
int whiteAtSlalomG = 66; // TournamentTODO
int whiteAtSlalomB = 54; // TournamentTODO
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN10;
#endif

// 黒（キャリブレータから上書きされるので設定しなくて良い）
int blackR = 6; // TournamentTODO
int blackG = 6; // TournamentTODO
int blackB = 6; // TournamentTODO
RawColorPredicateCondition blackRCondition = BETWEEN5;
RawColorPredicateCondition blackGCondition = BETWEEN5;
RawColorPredicateCondition blackBCondition = BETWEEN5;

// 赤
int redR = 67; // TournamentTODO
int redG = 24; // TournamentTODO
int redB = 13; // TournamentTODO
RawColorPredicateCondition redRCondition = BETWEEN5;
RawColorPredicateCondition redGCondition = BETWEEN5;
RawColorPredicateCondition redBCondition = BETWEEN5;

// 緑
int greenR = 12; // TournamentTODO
int greenG = 42; // TournamentTODO
int greenB = 13; // TournamentTODO
RawColorPredicateCondition greenRCondition = BETWEEN5;
RawColorPredicateCondition greenGCondition = BETWEEN5;
RawColorPredicateCondition greenBCondition = BETWEEN5;

// 青（キャリブレータから上書きされるので設定しなくて良い）
int blueR = 6;  // TournamentTODO
int blueG = 22; // TournamentTODO
int blueB = 34; // TournamentTODO
RawColorPredicateCondition blueRCondition = BETWEEN3;
RawColorPredicateCondition blueGCondition = BETWEEN3;
RawColorPredicateCondition blueBCondition = BETWEEN3;

// 黄
int yellowR = 75; // TournamentTODO
int yellowG = 70; // TournamentTODO
int yellowB = 10; // TournamentTODO
RawColorPredicateCondition yellowRCondition = BETWEEN5;
RawColorPredicateCondition yellowGCondition = BETWEEN5;
RawColorPredicateCondition yellowBCondition = BETWEEN5;

// グレー（キャリブレータから上書きされるので設定しなくて良い）
int grayR = 27; // TournamentTODO
int grayG = 40; // TournamentTODO
int grayB = 28; // TournamentTODO
RawColorPredicateCondition grayRCondition = BETWEEN5;
RawColorPredicateCondition grayGCondition = BETWEEN5;
RawColorPredicateCondition grayBCondition = BETWEEN5;

// 青白境界（キャリブレータから上書きされるので設定しなくて良い）//TODO エッジを実測して。（平均を取るのではダメらしい）
int blueWhiteEdgeR = 32; // TournamentTODO
int blueWhiteEdgeG = 54; // TournamentTODO
int blueWhiteEdgeB = 54; // TournamentTODO
RawColorPredicateCondition blueWhiteEdgeRCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeGCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeBCondition = BETWEEN10;
#endif

#ifdef RobotKomichi // TODO
// 白（キャリブレータから上書きされるので設定しなくて良い）
int whiteR = 69; // TournamentTODO
int whiteG = 66; // TournamentTODO
int whiteB = 81; // TournamentTODO
RawColorPredicateCondition WhiteRCondition = BETWEEN5;
RawColorPredicateCondition whiteGCondition = BETWEEN5;
RawColorPredicateCondition whiteBCondition = BETWEEN5;

// 黒白境界（キャリブレータから上書きされるので設定しなくて良い）
int blackWhiteEdgeR = 45; // TournamentTODO
int blackWhiteEdgeG = 52; // TournamentTODO
int blackWhiteEdgeB = 60; // TournamentTODO
RawColorPredicateCondition blackWhiteEdgeRCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeGCondition = BETWEEN5;
RawColorPredicateCondition blackWhiteEdgeBCondition = BETWEEN5;

// スラローム上からみた白（キャリブレータから上書きされるので設定しなくて良い）
#ifdef SimulatorMode
int whiteAtSlalomR = 115;
int whiteAtSlalomG = 110;
int whiteAtSlalomB = 155;
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN5;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN5;
#else
int whiteAtSlalomR = 64; // TournamentTODO
int whiteAtSlalomG = 61; // TournamentTODO
int whiteAtSlalomB = 83; // TournamentTODO
RawColorPredicateCondition whiteAtSlalomRCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomGCondition = BETWEEN10;
RawColorPredicateCondition whiteAtSlalomBCondition = BETWEEN10;
#endif

// 黒（キャリブレータから上書きされるので設定しなくて良い）
int blackR = 6; // TournamentTODO
int blackG = 6; // TournamentTODO
int blackB = 6; // TournamentTODO
RawColorPredicateCondition blackRCondition = BETWEEN5;
RawColorPredicateCondition blackGCondition = BETWEEN5;
RawColorPredicateCondition blackBCondition = BETWEEN5;

// 赤
int redR = 68; // TournamentTODO
int redG = 23; // TournamentTODO
int redB = 19; // TournamentTODO
RawColorPredicateCondition redRCondition = BETWEEN5;
RawColorPredicateCondition redGCondition = BETWEEN5;
RawColorPredicateCondition redBCondition = BETWEEN5;

// 緑
int greenR = 9;  // TournamentTODO
int greenG = 33; // TournamentTODO
int greenB = 18; // TournamentTODO
RawColorPredicateCondition greenRCondition = BETWEEN5;
RawColorPredicateCondition greenGCondition = BETWEEN5;
RawColorPredicateCondition greenBCondition = BETWEEN5;

// 青（キャリブレータから上書きされるので設定しなくて良い）
int blueR = 6;  // TournamentTODO
int blueG = 22; // TournamentTODO
int blueB = 54; // TournamentTODO
RawColorPredicateCondition blueRCondition = BETWEEN3;
RawColorPredicateCondition blueGCondition = BETWEEN3;
RawColorPredicateCondition blueBCondition = BETWEEN3;

// 黄
int yellowR = 70; // TournamentTODO
int yellowG = 57; // TournamentTODO
int yellowB = 15; // TournamentTODO
RawColorPredicateCondition yellowRCondition = BETWEEN5;
RawColorPredicateCondition yellowGCondition = BETWEEN5;
RawColorPredicateCondition yellowBCondition = BETWEEN5;

// グレー（キャリブレータから上書きされるので設定しなくて良い）
int grayR = 29; // TournamentTODO
int grayG = 40; // TournamentTODO
int grayB = 47; // TournamentTODO
RawColorPredicateCondition grayRCondition = BETWEEN5;
RawColorPredicateCondition grayGCondition = BETWEEN5;
RawColorPredicateCondition grayBCondition = BETWEEN5;

// 青白境界（キャリブレータから上書きされるので設定しなくて良い）//TODO エッジを実測して。（平均を取るのではダメらしい）
int blueWhiteEdgeR = 31; // TournamentTODO
int blueWhiteEdgeG = 46; // TournamentTODO
int blueWhiteEdgeB = 73; // TournamentTODO
RawColorPredicateCondition blueWhiteEdgeRCondition = BETWEEN20;
RawColorPredicateCondition blueWhiteEdgeGCondition = BETWEEN10;
RawColorPredicateCondition blueWhiteEdgeBCondition = BETWEEN10;
#endif

// 色設定ここまで

// ********** 設定2/2ここまで **********
