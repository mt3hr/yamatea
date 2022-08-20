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
#include "app.h"

#include "ev3api.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "SonarSensor.h"
#include "GyroSensor.h"

#include "string"

#include "Command.h"
#include "CommandExecutor.h"
#include "Predicate.h"
#include "Walker.h"
#include "Stopper.h"
#include "LeftWheelCountPredicate.h"

using namespace ev3api;

// apiで使うものの初期化
TouchSensor *touchSensor = new TouchSensor(PORT_1);
ColorSensor *colorSensor = new ColorSensor(PORT_2);
SonarSensor *sonarSensor = new SonarSensor(PORT_3);
GyroSensor *gyroSensor = new GyroSensor(PORT_4);
Motor *armMotor = new Motor(PORT_A);
Motor *rightWheel = new Motor(PORT_B);
Motor *leftWheel = new Motor(PORT_C);
Motor *tailMotor = new Motor(PORT_D);
Clock *clock = new Clock();

// Commandを実行するオブジェクト
CommandExecutor *commandExecutor;
// 走行体のAPIをまとめたオブジェクト
RobotAPI *robotAPI;

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
  Predicate *oneRotateLeftWheelPredicate = new LeftWheelCountPredicate(360);

  // 上で定義したウォーカと終了条件をcommandExecutorに追加する。
  commandExecutor->addCommand(walker, oneRotateLeftWheelPredicate);
}

// ここからいつものやつ

// 周期ハンドラトレーサタスクの定義
void runner_task(intptr_t exinf)
{
  ev3_lcd_draw_string("yamatea green tea", 0, 0);
  // 走って！！！
  commandExecutor->run();
  ext_tsk();
}

// メインタスクの定義
void main_task(intptr_t unused)
{
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  // フォントの設定と0行目の表示
  ev3_lcd_draw_string("yamatea green tea", 0, 0);
  robotAPI = new RobotAPI(touchSensor, colorSensor, sonarSensor, leftWheel, rightWheel, armMotor, gyroSensor, clock, tailMotor);

  // 初期化処理を呼び出す
  initialize();

  // 周期ハンドラを開始する
  sta_cyc(RUNNER_CYC);

  // 終了判定処理
  while (true)
  {
    // 左ボタンが押されたら緊急停止のためにループを抜ける
    if (ev3_button_is_pressed(LEFT_BUTTON))
    {
      // 停止処理
      commandExecutor->emergencyStop();
      break;
    }

    // RUNNER_CYCが終了していたら走行完了なのでループを抜ける
    T_RCYC pk_rcyc;
    ref_cyc(RUNNER_CYC, &pk_rcyc);
    if (pk_rcyc.cycstat == TCYC_STP)
    {
      Stopper *stopper = new Stopper();
      stopper->run(robotAPI);
      delete stopper;
      break;
    }

    // ちょっと待つ
    clock->sleep(100*1000);
  }

  // メインタスクの終了
  ext_tsk();

  // 終了処理。各オブジェクトの削除
  commandExecutor->emergencyStop();
  delete commandExecutor;
  delete robotAPI;
  delete touchSensor;
  delete colorSensor;
  delete sonarSensor;
  delete gyroSensor;
  delete armMotor;
  delete leftWheel;
  delete rightWheel;
  delete tailMotor;
  delete clock;
}

// ここまでいつものやつ

// ここから無視してOK
void listen_bluetooth_command_task(intptr_t exinf)
{
  ext_tsk();
}
void return_to_start_point_task(intptr_t exinf)
{
  ext_tsk();
}
// ここまで無視してOK
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