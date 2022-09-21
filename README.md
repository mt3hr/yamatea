# README

地区大会通過！
ETRobo2022 CS 出場決定やったー！！
学生賞、総合戦績 3 位やったー！！！

## yamatea

ETRobo2022 の走行プログラムです。  
やまとがお茶を飲みながら書いたプログラムなので yamatea。  
緑茶は美味しい。CS 大会優勝。

## yamatea の位置付け

~~小路がモデルを書いてくれていますが、沖原と私と時間の都合によって~~  
~~モデルに対応したコード完成が間に合いそうにないです。~~  
~~そのため、試走会 1 までに動くプログラムとして作りました。~~  
今後こいつで戦っていくぞ！

## 設計と使い方、サンプルコード

ロボットの動き（キャリブレーション、輝度ライントレース、色ライントレース、モータパワー指定走行、ロボット旋回など）を Command クラスに抽象化しました。  
ロボットを動かすときには Command クラスのサブクラスを定義し、それを CommandExecutor.addCommand(command)で追加します。

app.cpp のサンプルコードを載せておきます。  
実行するとロボットが50cm直進します。  
このプロジェクトの app.cpp を上書きすれば動くかもしれないです。  
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
#include "WheelDistancePredicate.h"
#include "PrintMessage.h"
#include "Stopper.h"
#include "StartCyc.h"

using namespace ev3api;

// Commandを実行するオブジェクト
CommandExecutor *commandExecutor;
// 走行体のAPIをまとめたオブジェクト
RobotAPI *robotAPI;

// commandExecutorなどの初期化処理。
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
    // シンプルなウォーカ。左右車輪50のpwmで進む。
    int leftPow = 50;
    int rightPow = 50;
    int distance = 50;
    Command *walker = new Walker(leftPow, rightPow);
    string commandName = "walker";

    // ウォーカを終了するタイミングを決定するPredicate
    // この例では50cm進んだとき
    Predicate *distance50cmPredicate = new WheelDistancePredicate(distance, robotAPI);

    // 上で定義したウォーカと終了条件をcommandExecutorに追加する。
    commandExecutor->addCommand(walker, distance50cmPredicate, commandName);
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
    ev3_lcd_draw_string("yamatea green tea", 0, 0);

    const uint32_t sleepDuration = 100 * 1000;
    const int8_t line_height = 20;

    // 初期化中メッセージの出力
    int line = 1;
    ev3_lcd_fill_rect(0, line * line_height, EV3_LCD_WIDTH, line_height, EV3_LCD_WHITE);
    ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示
    ev3_lcd_draw_string("initializing...", 0, line * line_height);

    // EV3APIオブジェクトの初期化
    TouchSensor *touchSensor = new TouchSensor(PORT_1);
    ColorSensor *colorSensor = new ColorSensor(PORT_2);
    SonarSensor *sonarSensor = new SonarSensor(PORT_3);
    GyroSensor *gyroSensor = new GyroSensor(PORT_4);
    Motor *armMotor = new Motor(PORT_A);
    Motor *rightWheel = new Motor(PORT_B);
    Motor *leftWheel = new Motor(PORT_C);
    Motor *tailMotor = new Motor(PORT_D);
    Clock *clock = new Clock();

    // RobotAPIとCommandExecutorの初期化
    robotAPI = new RobotAPI(touchSensor, colorSensor, sonarSensor, leftWheel, rightWheel, armMotor, tailMotor, gyroSensor, clock);
    commandExecutor = new CommandExecutor(robotAPI, true);

    // ev3_lcd_set_font(EV3_FONT_MEDIUM);           // フォントの設定
    ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示

    // robotAPIの初期化。完全停止してapiを初期化する
    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    robotAPI->getClock()->sleep(sleepDuration);
    robotAPI->reset();
    robotAPI->getClock()->sleep(sleepDuration);
    delete stopper;
    vector<string> resetedMessageLines;
    resetedMessageLines.push_back("reseted api");
    PrintMessage *printResetedMessage = new PrintMessage(resetedMessageLines, true);
    printResetedMessage->run(robotAPI);
    delete printResetedMessage;
    robotAPI->getClock()->sleep(sleepDuration);

    // commandExecutorを初期化する（挙動定義）
    initializeCommandExecutor(commandExecutor, robotAPI);
    vector<string> readyMessageLines;
    readyMessageLines.push_back("ready");
    PrintMessage *printReadyMessage = new PrintMessage(resetedMessageLines, true);
    printReadyMessage->run(robotAPI);
    delete printReadyMessage;

    // commandExecutor->run()の周期ハンドラを起動する
    StartCyc *startRunnerCyc = new StartCyc(RUNNER_CYC);
    startRunnerCyc->run(robotAPI);
    delete startRunnerCyc;

    // 終了判定処理
    for (; true; clock->sleep(sleepDuration))
    {
        // 左ボタンが押されたら緊急停止のためにループを抜ける
        if (ev3_button_is_pressed(LEFT_BUTTON))
        {
            // 停止処理
            commandExecutor->emergencyStop();
        }

        // RUNNER_CYCが終了していたら走行完了なのでループを抜ける
        // CommandExecutorが終了していたら走行完了なのでループを抜ける
        if (commandExecutor->isFinished())
        {
            break;
        }
    }

    // runner_cycを止める
    stp_cyc(RUNNER_CYC);

    // 走行体停止
    stopper->run(robotAPI);
    delete stopper;

    // 終了メッセージの表示
    robotAPI->getClock()->sleep(sleepDuration);
    vector<string> messageLines;
    messageLines.push_back("finish!!");
    PrintMessage printFinishMessage(messageLines, true);
    printFinishMessage.run(robotAPI);

    // メインタスクの終了
    ext_tsk();

    // オブジェクトの削除
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

void sing_a_song_task(intptr_t exinf)
{
    ext_tsk();
}

void dededon_task(intptr_t exinf)
{
    ext_tsk();
}
// ここまで無視してOK
```

Command は Walker の他に PIDTracer や、  
それのための PIDTargetBrightnessCalibrator があります。

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

かいたひと: 日本電子専門学校 情報システム開発科 21 年度学生 実方大和（さねかたやまと）
