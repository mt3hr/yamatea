# yamatea_manual

## yamateaのマニュアル
マニュアル？Q&A？  

## 本番用設定
### 左コース
設定1/2（Setting.h）  
```h
// ********** 設定1/2ここから **********

// 実機シミュレータ設定。ジャイロセンサから取得できる角度の方向が実機とシミュレータでは異なるので。
//#define SimulatorMode // 実機で動かすときにはコメントアウトして

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
#define LeftCourceMode // 左コース用プログラム
//#define RightCourceMode // 右コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode    // RGBRawの値をはかるプログラム
//#define Rotate360TestMode // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
//#define RotateTestMode // 旋回モード。テスト用
//#define RotateGyroTestMode // ジャイロを使った旋回モード。テスト用。
//#define StraightTestMode // 直進モード。テスト用
//#define CurvatureWalkerTestMode // 曲率旋回モード。テスト用
//#define SwingSonarDetectorTestMode // 障害物距離角度首振り検出モード。テスト用
//#define ShigekiTestMode // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
// #define UFORunnerTestMode // UFO走行モード。テスト
// モード設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ

// ********** 設定1/2ここまで **********
```

設定2/2（Setting.cpp）  
```cpp
// ********** 設定2/2ここから **********

// 車体情報設定ここから
float wheelDiameter = 10.4;// 車輪直径。センチメートル。
float distanceFromSonarSensorToAxle = 10.5; // ソナーセンサから車軸までの距離
float wheelSpace = 14.5; // 左車輪と右車輪の間隔
int angleFor360TurnLeftRotateRobot = 520; // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数
// 車体情報設定ここまで

// 情報出力の有効無効設定ここから
DEBUG_LEVEL debugMessageLevel = NONE;   // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = false; // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForConsole = false;  // trueにすると、コンソールにも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）
bool enablePrintMessageForBluetooth = false; // trueにすると、Bluetooth接続端末にも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）trueにする場合、ヘッダファイルの#define EnableBluetoothのコメントアウトも外して。
// 情報出力の有効無効設定ここまで

// ********** 設定2/2ここまで **********
```

### 右コース
設定1/2（Setting.h）  
```h
// ********** 設定1/2ここから **********

// 実機シミュレータ設定。ジャイロセンサから取得できる角度の方向が実機とシミュレータでは異なるので。
//#define SimulatorMode // 実機で動かすときにはコメントアウトして

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
//#define LeftCourceMode // 左コース用プログラム
#define RightCourceMode // 右コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode    // RGBRawの値をはかるプログラム
//#define Rotate360TestMode // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
//#define RotateTestMode // 旋回モード。テスト用
//#define RotateGyroTestMode // ジャイロを使った旋回モード。テスト用。
//#define StraightTestMode // 直進モード。テスト用
//#define CurvatureWalkerTestMode // 曲率旋回モード。テスト用
//#define SwingSonarDetectorTestMode // 障害物距離角度首振り検出モード。テスト用
//#define ShigekiTestMode // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
// #define UFORunnerTestMode // UFO走行モード。テスト
// モード設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ

// ********** 設定1/2ここまで **********
```

設定2/2（Setting.cpp）  
```cpp
// ********** 設定2/2ここから **********

// 車体情報設定ここから
float wheelDiameter = 10.4;// 車輪直径。センチメートル。
float distanceFromSonarSensorToAxle = 10.5; // ソナーセンサから車軸までの距離
float wheelSpace = 14.5; // 左車輪と右車輪の間隔
int angleFor360TurnLeftRotateRobot = 520; // 左に360度旋回するのに必要な左右車輪回転角度数
int angleFor360TurnRightRotateRobot = 510; // 右に360度旋回するのに必要な左右車輪回転角度数
// 車体情報設定ここまで

// 情報出力の有効無効設定ここから
DEBUG_LEVEL debugMessageLevel = NONE;   // 出力するデバッグ情報のレベル。None, Info, Debug, Trace。
bool enablePrintMessageMode = false; // trueにすると、コマンドの情報をディスプレイなどに表示する。ただし、ディスプレイ表示処理は重いので走行が変わる。enablePrintMessageForConsole, enablePrintMessageForConsole, enablePrintMessageForBluetoothを有効化するならばこの値も有効化して。
bool enablePrintMessageForConsole = false;  // trueにすると、コンソールにも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）
bool enablePrintMessageForBluetooth = false; // trueにすると、Bluetooth接続端末にも情報がprintされる。（PrintMessageModeのコメントアウトを外す必要がある）trueにする場合、ヘッダファイルの#define EnableBluetoothのコメントアウトも外して。
// 情報出力の有効無効設定ここまで

// ********** 設定2/2ここまで **********
```

## yamateaの設計と使い方
### yamateaの基礎概念（？）
- Command: ロボットの挙動（例: pid走行（PIDTracer）, アーム上げ下げ（ArmController））  
- CommandExecutor: 与えられたコマンドを逐次実行する実行者。コマンドを渡すときには終了条件も添え、その条件を満たしたときに次のコマンドへ遷移する。  
- Predicate: 終了条件（例: モータ回転角度が指定角度を超えた（MotorCountPredicate）, ジャイロセンサの示す角度が指定角度を超えた（GyroRotateAnglePredicate））  

### yamateaの使い方
サンプルプログラムについては[README.md](https://github.com/mt3hr/yamatea/blob/master/README.md)を参照してください。  

## 基本的なこと

### このプログラムを使う用意
StartETRobo.batから環境を起動している前提で説明します。  

#### 初回ダウンロード（git clone）
1. 次のコマンドを実行する  
```bash
cd ~/workspace;
git clone git@github.com:mt3hr/yamatea.git;
```
1. workspaceフォルダ内にyamateaディレクトリが作られるので、作業する  

#### 更新適用（git pull）
初回ダウンロード（git clone）が済んでいる前提で説明します。  
1. 次のコマンドを実行する  
```bash
cd ~/workspace/yamatea;
git add *;
git commit -m "update"; # -mはコミットメッセージなので必要に応じて変えて。
git pull;
```
1. なんかエディタっぽいのが起動した場合は、「CTRL+S」を押して、一旦キーを離し、「CTRL+S」を押す  

### プログラム変更時にやること
StartETRobo.batから環境を起動している前提で説明します。  

#### 実機に転送する場合
1. 変更したプログラムファイルを保存する  
1. PCとEV3をケーブルで接続する  
1. VSCodeで次のコマンドを入力し、実行する  
```bash
make app=yamatea up
```
1. PCとEV3からケーブルを取り外す  
1. EV3でyamateaを選択して実行する（大体のプログラムはロボット左腕のタッチセンサーを押すと動きはじめます）  

#### シミュレータで試走する場合
1. 変更したプログラムファイルを保存する  
1. VSCodeで次のコマンドを入力し、実行する  
```bash
make app=yamatea sim up
```
1. 待つ（動き出すまでに結構時間がかかる）  

## 動かない、期待通りの動きをしない

設定ミスの可能性が高いです。  
原因別に説明します。  

### モード設定が誤っている
設定1/2（Setting.h）にモード設定があります。  
1つだけ有効化して、他をコメントアウトしてください。  
ロボットは有効化されたモードの挙動をします。  

### 「SimulatorMode」 を変更し忘れている
2022-08-19現在、実機とシミュレータとでジャイロセンサから取得できる角度の符号が異なります。  
それに対応するための設定「SimulatorMode」があります。  
設定し忘れていると、GyroRotateAnglePredicateなどが期待どおりに動かないです。  
（内部でそれを使ったクラス、例えばUFORunnerなども期待どおりに動きません）  