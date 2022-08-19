# README

## yamatea
ETRobo2022の走行プログラムです。  
やまとがお茶を飲みながら書いたプログラムなのでyamatea。  
緑茶は美味しい。CS大会優勝。  

## yamateaの位置付け
~~小路がモデルを書いてくれていますが、沖原と私と時間の都合によって~~  
~~モデルに対応したコード完成が間に合いそうにないです。~~  
~~そのため、試走会1までに動くプログラムとして作りました。~~  
今後こいつで戦っていくぞ！  

## 設計と使い方、サンプルコード
ロボットの動き（輝度キャリブレーション、ライントレース、モータパワー指定走行など）をCommandクラスに抽象化しました。  
ロボットを動かすときにはCommandクラスのサブクラスを定義し、それをCommandExecutor.addCommand(command)で追加します。  

app.cppのサンプルコードを載せておきます。  
実行するとロボットが車輪一回転分直進します。  
このプロジェクトのapp.cppを上書きすれば動くかもしれないです。  
もし動かなかったら切腹します。  

```c++:app.cpp
#include "Motor.h"
#include "Clock.h"

#include "app.h"

#include "Command.h"
#include "CommandExecutor.h"
#include "Predicate.h"
#include "Handler.h"
#include "WheelController.h"
#include "Walker.h"
#include "MotorCountPredicate.h"

#include "string"

using namespace ev3api;

// apiで使うものの初期化
TouchSensor *touchSensor = new TouchSensor(PORT_1);
ColorSensor *colorSensor = new ColorSensor(PORT_2);
SonarSensor *sonarSensor = new SonarSensor(PORT_3);
Motor *leftWheel = new Motor(PORT_C);
Motor *rightWheel = new Motor(PORT_B);
Motor *armMotor = new Motor(PORT_A);
Clock *clock = new Clock();

// Commandを実行するオブジェクト
CommandExecutor *commandExecutor;
// 走行体のAPIをまとめたオブジェクト
RobotAPI *robotAPI = new RobotAPI(touchSensor, colorSensor, sonarSensor, leftWheel, rightWheel, armMotor, clock);

// サンプルのutil.cppから引っ張ってきたやつ
// char*ではなくstd::stringで受け取る
void init_f(string str)
{
  // フォントの設定と0行目の表示
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string(str.c_str(), 0, 0);
}

// commandExecutorなどの初期化処理。
void initialize()
{
  // commandExecutorとwheelControllerの初期化
  commandExecutor = new CommandExecutor(robotAPI);

  // シンプルなウォーカ。左右車輪50のpwmで進む。
  int leftPow = 50;
  int rightPow = 50;
  Command *walker = new Walker(leftPow, rightPow);

  // ウォーカを終了するタイミングを決定するPredicate
  // この例では左車輪が360度回転したら終了する
  Predicate *oneRotateLeftWheelPredicate = new MotorCountPredicate(leftWheel, 360);

  // コマンド終了時に走らされるハンドラ。この例ではなにもしない。
  Handler *doNothingExitHandler = new Handler();

  // 上で定義したウォーカと終了条件をcommandExecutorに追加する。
  commandExecutor->addCommand(walker, oneRotateLeftWheelPredicate, doNothingExitHandler);
}

// ここからいつものやつ

// 周期ハンドラトレーサタスクの定義
void tracer_task(intptr_t exinf)
{
  init_f(string("yamatea green tea"));
  // 走って！！！
  commandExecutor->run();
  ext_tsk();
}

// メインタスクの定義
void main_task(intptr_t unused)
{
  sta_cyc(TRACER_CYC);

  // 初期化処理を呼び出す
  initialize();

  while (!ev3_button_is_pressed(LEFT_BUTTON))
  {
    clock->sleep(100000);
  }

  stp_cyc(TRACER_CYC);

  // 終了処理。各オブジェクトの削除
  commandExecutor->emergencyStop();
  delete commandExecutor;
  delete robotAPI;
  delete touchSensor;
  delete colorSensor;
  delete sonarSensor;
  delete leftWheel;
  delete rightWheel;
  delete clock;
  ext_tsk();
}

// ここまでいつものやつ
```
CommandはWalkerの他にPIDTracerや、  
それのためのPIDTargetBrightnessCalibratorがあります。  

以下細かいことを書きま　　　　せん。  
書こうと思いましたがやめます。  
ヘッダファイルに雑なコメントを残しましたし、モデルも書きました。  
モデルを見てください。どこかにあるので（多分クラス図しか無いけど）  
冗長、そしてキャメルケースという観点からは読み辛いコードかもしれないけど、私頑張ったから。  
えらいね、ありがとう。  

## リンク
- [yamatea_manual](https://github.com/mt3hr/yamatea/blob/master/yamatea_manual.md)

## 終わり
この文章を読んだ人は今後頑張らなければなりません。  
頑張って。  

かいたひと: 日本電子専門学校 情報システム開発科 21年度学生 実方大和（さねかたやまと）  