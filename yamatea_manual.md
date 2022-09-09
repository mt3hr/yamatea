# yamatea_manual

## yamatea のマニュアル

マニュアル？Q&A？

## 本番用設定

<details>
<summary>左コース</summary>

設定 1/2（Setting.h）

```h
// ********** 設定1/2ここから **********

// 実機シミュレータ設定。ジャイロセンサから取得できる角度の方向が実機とシミュレータでは異なるので。
//#define SimulatorMode // 実機で動かすときにはコメントアウトして

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
#define LeftCourceMode // 左コース用プログラム
//#define RightCourceMode // 右コース用プログラム
//#define SlalomUFOTestMode // スラロームをUFO走行するプログラム。
//#define SlalomAwaitingSignalModePattern1_1 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1
//#define SlalomAwaitingSignalModePattern2_1 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2
//#define SlalomAwaitingSignalModePattern1_2 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1
//#define SlalomAwaitingSignalModePattern2_2 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2
//#define SlalomAwaitingSignalModePattern1_3 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1
//#define SlalomAwaitingSignalModePattern2_3 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2
//#define BlockTestMode  // ブロック搬入だけを走行するプログラム。
//#define FlatLineMode // すべて同じPIDで倉庫する左コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode // RGBRawの値をはかるプログラム
//#define ColorIDReaderMode // ColorIDを取得し続けるプログラム
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

// モード設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ
//#define SingASong       // 走行時に歌う

// ********** 設定1/2ここまで **********
```

設定 2/2（Setting.cpp）

```cpp
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
```

</details>

<details>
<summary>右コース</summary>

設定 1/2（Setting.h）

```h
// ********** 設定1/2ここから **********

// 実機シミュレータ設定。ジャイロセンサから取得できる角度の方向が実機とシミュレータでは異なるので。
//#define SimulatorMode // 実機で動かすときにはコメントアウトして

// モード設定ここから
// どれか一つを有効化して、それ以外をコメントアウトしてください
//#define LeftCourceMode // 左コース用プログラム
#define RightCourceMode // 右コース用プログラム
//#define SlalomUFOTestMode // スラロームをUFO走行するプログラム。
//#define SlalomAwaitingSignalModePattern1_1 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1
//#define SlalomAwaitingSignalModePattern2_1 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2
//#define SlalomAwaitingSignalModePattern1_2 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1
//#define SlalomAwaitingSignalModePattern2_2 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2
//#define SlalomAwaitingSignalModePattern1_3 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン1
//#define SlalomAwaitingSignalModePattern2_3 // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。パターン2
//#define BlockTestMode  // ブロック搬入だけを走行するプログラム。
//#define FlatLineMode // すべて同じPIDで倉庫する左コース用プログラム
//#define DistanceReaderMode // 距離をはかり続けるプログラム
//#define RGBRawReaderMode // RGBRawの値をはかるプログラム
//#define ColorIDReaderMode // ColorIDを取得し続けるプログラム
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

// モード設定ここまで

//#define EnableBluetooth // enablePrintMessageForBluetoothをtrueにする場合はこれのコメントアウトも外して。// いらないかもなこれ
//#define SingASong       // 走行時に歌う

// ********** 設定1/2ここまで **********
```

設定 2/2（Setting.cpp）

```cpp
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
```

</details>

## yamatea の設計と使い方

### yamatea の基礎概念（？）

- Command: ロボットの挙動（例: pid 走行（PIDTracer）, アーム上げ下げ（ArmController））
- CommandExecutor: 与えられたコマンドを逐次実行する実行者。コマンドを渡すときには終了条件も添え、その条件を満たしたときに次のコマンドへ遷移する。
- Predicate: 終了条件（例: モータ回転角度が指定角度を超えた（MotorCountPredicate）, ジャイロセンサの示す角度が指定角度を超えた（GyroRotateAnglePredicate））

### yamatea の使い方

サンプルプログラムについては[README.md](https://github.com/mt3hr/yamatea/blob/master/README.md)を参照してください。

## 基本的なこと

### このプログラムを使う用意

StartETRobo.bat から環境を起動している前提で説明します。

#### 初回ダウンロード（git clone）

1. 次のコマンドを実行する

```bash
cd ~/etrobo/workspace; git clone git@github.com:mt3hr/yamatea.git;
```

1. workspace フォルダ内に yamatea ディレクトリが作られるので、作業する

#### 更新適用（git pull）

初回ダウンロード（git clone）が済んでいる前提で説明します。

1. 次のコマンドを実行する

```bash
cd ~/etrobo/workspace/yamatea; git add *; git commit -m "update"; git pull;
```

1. なんかエディタっぽいのが起動した場合は、「CTRL+S」を押して、一旦キーを離し、「CTRL+S」を押す

### プログラム変更時にやること

StartETRobo.bat から環境を起動している前提で説明します。

#### 実機に転送する場合

1. 変更したプログラムファイルを保存する
1. PC と EV3 をケーブルで接続する
1. VSCode で次のコマンドを入力し、実行する

```bash
make app=yamatea up
```

1. PC と EV3 からケーブルを取り外す
1. EV3 で yamatea を選択して実行する（大体のプログラムはロボット左腕のタッチセンサーを押すと動きはじめます）

#### シミュレータで試走する場合

1. 変更したプログラムファイルを保存する
1. VSCode で次のコマンドを入力し、実行する

```bash
make app=yamatea sim up
```

1. 待つ（動き出すまでに結構時間がかかる）

### 新しい挙動の記述方法（モード追加方法）

流れは次のとおりです。下で具体的な記述方法を説明します。

1. Setting.h にモードを定義する
1. app.cpp に追加したモードの initializeCommandExecutor()関数を定義する
1. Setting.h の他のモードをコメントアウトする
1. ビルドする

#### 1. Setting.h にモードを定義する

Setting.h に追加したいモードを定義します。
例えば、直進し続けるモード「StraightMode」を追加するには次のようにします。

変更前

```c:Setting.h
// モード設定ここから
#define LeftCourceMode
//#define RightCourceMode
// モード設定ここまで
```

変更後

```c:Setting.h
// モード設定ここから
#define LeftCourceMode
//#define RightCourceMode
#define StraightMode
// モード設定ここまで
```

#### 2. app.cpp に追加したモードの initializeCommandExecutor()関数を定義する

変更前

```c++:app.cpp
// ↑省略
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
    // 省略
}
#endif
// ↓省略
```

変更後

```c++:app.cpp
// ↑省略
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
    // 省略
}
#endif

// ここから追加
#if defined(StraightMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, GET_VARIABLE_NAME(stopper));

  int pwm = 50;
  float distanceCm = 10000;
  Walker *walker = new Walker(pwm, pwm);
  DistancePredicate *walkerPredicate = new DistancePredicate(distanceCm, robotAPI);
  commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));

  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif
// ここまで追加
// ↓省略
```

#### 3. Setting.h の他のモードをコメントアウトする

モード設定で、起動するモード以外の行をコメントアウトします。
変更前

```c:Setting.h
// モード設定ここから
#define LeftCourceMode
//#define RightCourceMode
#define StraightMode
// モード設定ここまで
```

変更後

```c:Setting.h
// モード設定ここから
//#define LeftCourceMode   // ←コメントアウト
//#define RightCourceMode
#define StraightMode
// モード設定ここまで
```

#### 4. ビルドする

ビルドして通れば OK

#### モード管理でこのやりかたをする理由

これがいいとは思っていないけれど、  
コメントアウトで切り替えができるので一番マシな気がして。  
app.cpp のファイル分割はしない予定です。（include が面倒くさすぎるため、また、ファイルが増えすぎるからです。C++、ヘッダファイルも書かないといけないから。）  
ファイル内検索を使って頑張りましょう。

### 設計モデルと実装の差

#### Command などのコンストラクタ引数と Command.run()メソッドなどの引数

Command のサブクラスは、EV3API オブジェクト（例えば ColorSensor など）をコンストラクタ引数として受け取らなくなりました。  
かわりに、Command.run()メソッドなどの引数に、RobotAPI が追加されました。  
RobotAPI は、EV3API オブジェクトを集約したものです。センサ, モータなどは RobotAPI の各種 getter から取得できます。

うまく説明できないですが、つまりどういうことかというと、  
Command のサブクラスは EV3API オブジェクトを所有しなくなりました。  
実行時、準備時にしか使わないなら関連減らしたほうがよくない？という考えによるものです。フィールドに埋め込まなくても実現できるなら、引数で渡したほうがいいよな。

モデルとコードがきれいになるというメリットがあります。

実行時以外に API アクセスが必要な Command は、コンストラクタ引数に RobotAPI を追加して対処しましょ。

#### パッケージ名

パッケージ名が異なります。  
また、command パッケージは難所ごとにサブパッケージが設けられています。

## 動かない、期待通りの動きをしない

設定ミスの可能性が高いです。  
原因別に説明します。

### ケーブルがささりきっていない

アプリケーション読み込みまで進んで、キャリブレータが起動しない場合。
各種センサのケーブルが刺さりきっていない可能性があります。

### モード設定が誤っている

設定 1/2（Setting.h）にモード設定があります。  
1 つだけ有効化して、他をコメントアウトしてください。  
ロボットは有効化されたモードの挙動をします。

### 「SimulatorMode」 を変更し忘れている

2022-08-19 現在、実機とシミュレータとでジャイロセンサから取得できる角度の符号が異なります。  
それに対応するための設定「SimulatorMode」があります。  
設定し忘れていると、GyroRotateAnglePredicate などが期待どおりに動かないです。  
（内部でそれを使ったクラス、例えば UFORunner なども期待どおりに動きません）

### 「EnableBluetooth」を有効化しているのに Bluetooth 接続していない

Bluetooth 接続しない場合はコメントアウトしてください。
期待どおりの走行をしなくなります。
なんかよくわからない場合は、この文書の上の方にある本番用左コース設定で動かしてみてください
