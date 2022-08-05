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
#include "util.h"

#include "Command.h"
#include "CommandExecutor.h"
#include "Predicate.h"
#include "Handler.h"
#include "WheelController.h"
#include "Walker.h"
#include "MotorCountPredicate.h"

using namespace ev3api;

// apiで使うものの初期化
Clock *clock = new Clock();
Motor *leftWheel = new Motor(PORT_C);
Motor *rightWheel = new Motor(PORT_B);

// Commandを実行するオブジェクト
CommandExecutor *commandExecutor;

// commandExecutorなどの初期化処理。
void initialize()
{
  // commandExecutorとwheelControllerの初期化
  commandExecutor = new CommandExecutor();
  WheelController *wheelController = new WheelController(leftWheel, rightWheel);

  // シンプルなウォーカ。左右車輪20のpwmで進む。
  int leftPow = 20;
  int rightPow = 20;
  Command *walker= new Walker(leftPow, rightPow, wheelController);

  // ウォーカを終了するタイミングを決定するPredicator
  // この例では左車輪が360度回転したら終了する
  Predicate *oneRotatePredicate = new MotorCountPredicate(leftWheel, 360);

  // コマンド終了時に走らされるハンドラ。この例ではなにもしない。
  Handler *donothingExitHandler = new Hanler();

  // 上で定義したウォーカと終了条件をcommandExecutorに追加する。
  commandExecutor->addCommand(walker, oneRotatePredicate, donothingExitHanler);
}

// ここからいつものやつ

// 周期ハンドラトレーサタスクの定義
void tracer_task(intptr_t exinf)
{
  init_f("yamatea green tea");
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
    clock.sleep(100000);
  }

  stp_cyc(TRACER_CYC);

  // 終了処理。各オブジェクトの削除
  commandExecutor->emergencyStop();
  delete commandExecutor;
  delete clock;
  delete leftWheel;
  delete rightWheel;

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

メモリ管理・・・？日本語でおｋ  

## 終わり
この文章を読んだ人は今後頑張らなければなりません。  
頑張って。  