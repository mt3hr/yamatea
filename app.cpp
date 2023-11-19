// 設定は2箇所に分散しています。
// 設定1/2: Setting.h
// 設定2/2: Setting.cpp
// 実方
#include "app.h"
#include "Setting.h"

#include "ev3api.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "SonarSensor.h"
#include "GyroSensor.h"

#include "sstream"
#include "vector"
#include "string"
#include "math.h"
#include "exception"

#include "PrintMessage.h"
#include "Command.h"
#include "CommandExecutor.h"
#include "Predicate.h"
#include "ANDPredicate.h"
#include "ORPredicate.h"
#include "PIDTracer.h"
#include "PIDTracerV2.h"
#include "PIDTracerV2.h"
#include "PIDStraightWalker.h"
#include "Walker.h"
#include "ArmController.h"
#include "ColorPredicate.h"
#include "DistanceReader.h"
#include "StartButtonPredicate.h"
#include "MotorCountPredicate.h"
#include "CommandAndPredicate.h"
#include "MotorRotateAnglePredicate.h"
#include "NumberOfTimesPredicate.h"
#include "Stopper.h"
#include "RGBRawReader.h"
#include "WheelDistancePredicate.h"
#include "RotateRobotCommandAndPredicate.h"
#include "FinishedCommandPredicate.h"
#include "CurvatureWalkerCommandAndPredicate.h"
#include "SwingSonarObstacleDetector.h"
#include "UFORunner.h"
#include "RobotAPI.h"
#include "GyroRotateAnglePredicate.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "ColorReaderUseRaw.h"
#include "CardColorReaderUseRaw.h"
#include "DebugUtil.h"
#include "Bluetooth.h"
#include "ColorPIDTracer.h"
#include "ColorPIDTracerV2.h"
#include "ColorPIDTracerV2.h"
#include "PIDTargetColorBrightnessCalibrator.h"
#include "TailController.h"
#include "MusicalScore.h"
#include "StartCyc.h"
#include "RawColorPredicate.h"
#include "ColorIDReader.h"
#include "FacingAngleAbs.h"
#include "FacingAngle.h"
#include "PIDFacingAngleAbs.h"
#include "PIDFacingAngle.h"
#include "ResetGyroSensor.h"
#include "ResetMeasAngle.h"
#include "Hedgehog.h"
#include "HedgehogUsePID.h"
#include "BatteryPredicate.h"
#include "RotateRobotCommandAndPredicateV2.h"
#include "FacingRobotUseWheelPredicate.h"
#include "DealingWithGarage.h"
#include "TimerPredicate.h"
#include "BrightnessReader.h"
#include "ResetArmAngle.h"
#include "ReleaseWheel.h"
#include "SonarDistancePredicate.h"
#include "AngleAbsPredicate.h"
#include "SetPWMCoefficient.h"
#include "ResetPWMCoefficient.h"
#include "WalkerR.h"
#include "PIDLimTracer.h"
#include "BatteryEaterSilent.h"

// initializeCommandExecutorの定義読み込みここから
#include "GoalSanekataPIDMode1.h"
#include "DistanceReaderMode.h" // 距離をはかり続けるプログラム。試走会用
#include "FlatLineMode.h" // すべて同じPIDで走行する左コース用プログラム
#include "GoalOkiharaPIDMode.h" // ゴール用プログラム。沖原作業用
#include "GoalOkiharaPIDMode1.h" // ゴール用プログラム。試走会1時点。 // ゴール用プログラム。地区大会時点安定
#include "GoalOkiharaPIDMode2.h" // GoalOkihara3Distanceをもとに補正値Rで爆速にしたもの // ゴール用プログラム
#include "GoalKomichiScnenarioTestMode.h" // 完全シナリオ小路コード
#include "GoalOkiharaPIDMode3.h" // ゴール用プログラム。1のスピードアップ版2022-10-18
#include "GoalOkiharaPIDMode4.h" // ゴール用プログラム。1のスピードアップ版2022-11-09
#include "GoalOkiharaPIDMode3Distance.h" // GoalOkihara3の距離計算をDistanceに書き換えたもの。実方自宅調節用
#include "GoalSanekataPIDMode2.h"
#include "GoalSanekataPIDMode3.h"
#include "GoalSanekataPIDMode2_Stable1.h"
#include "GoalSanekataPIDMode2_Stable2.h"
#include "SlalomUFOTestMode.h" // スラロームだけをUFO走行するプログラム。
#include "SlalomAwaitingSignalPlan1SanekataMode.h" // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案1
#include "SlalomAwaitingSignalPlan2SanekataMode.h" // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案2
#include "SlalomAwaitingSignalPlan3SanekataMode.h" // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案3
#include "SlalomAwaitingSignalPlan4SanekataMode.h" // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案4
#include "SlalomAwaitingSignalPlan5SanekataMode.h" // 青ラインからスラローム終わりまで指示待ちで走行するプログラム。案5
#include "BlockTestMode.h" // ブロック搬入だけを走行するプログラム。小路作業用
#include "ColorIDReaderMode.h" // ColorIDを取得し続けるプログラム。試走会用
#include "Rotate360TestMode.h" // Rotate360TestModeの場合のcommandExecutor初期化処理  // 360度回転に必要なモータ回転角をはかるためのもの。テスト用
#include "RotateTestMode.h" // 旋回モード。テスト用
#include "RotateGyroTestMode.h" // NOTE ジャイロ、 実機とシミュレータで左右判定が逆になる？  // ジャイロを使った旋回モード。テスト用。
#include "StraightTestMode.h"  // 直進モード。テスト用
#include "CurvatureWalkerTestMode.h" // 曲率旋回モード。テスト用
#include "SwingSonarDetectorTestMode.h" // 障害物距離角度首振り検出モード。テスト用
#include "ShigekiTestMode.h" // あなたの墓地にあり伝説でないカードＸ枚を対象とする。それらをあなたの手札に戻す。テスト用
#include "UFORunnerSwingTestMode.h" // UFO走行モード。障害物間を向いている状態から始める。テスト用
#include "UFORunnerClockwiseTestMode.h" // UFO走行モード。左障害物を向いている状態から始める。テスト用
#include "BrightnessPIDTracerTestMode.h" // TargetBrightnessPIDTraceを試すモード。テスト用
#include "OCCircleCourceMode.h" // オープンキャンパス参加者0人時暇つぶし円コース
#include "BrightnessPIDTracerV2TestMode.h" // BrightnessPIDTraceV2を試すモード。テスト用
#include "PIDLimTracerTestMode.h"
#include "ColorPIDTracerTestMode.h"  // ColorPIDTraceを試すモード。テスト用
#include "ColorPIDTracerV2TestMode.h" // ColorPIDTraceV2を試すモード。テスト用
#include "WalkerRTestMode.h"
#include "GrayPredicateTestMode.h" // グレーでとまる直進モード。テスト用。
#include "ColorReaderUseRawTestMode.h" // RawColorで色を取得し続けるプログラム。試走会用。
#include "FroggySongTestMode.h" // かえるの歌を歌わせるモード。テスト用。
#include "FreedomDiveTestMode.h"
#include "FacingAngleTestMode.h" // 指定角度に向き直るモード。テスト用。
#include "WalkerTestMode.h" // Walkerで走るモード。テスト用。
#include "BatteryEaaterMode.h" // その場旋回をして電池を消費するモード。テスト用
#include "BatteryEeaterSilentMode.h"
#include "SlalomBlockJoinTestMode.h"
#include "PIDStraightWalkerTestMode.h" // PIDStraightWalkerをテストするモード
#include "TrueCourceOkiharaModeRegional.h" // 地区大会完走用プログラム。沖原トレース
#include "TrueCourceKomichiModeRegional.h" // 地区大会完走用プログラム。小路シナリオ
#include "TrueCourceSanekataModeCS.h" // CS完走用プログラム
#include "TrueCourceSanekataMode1261b.h" // CS後1261b教室用に調節した完走用プログラム。7823mv、Lコースパターン1
#include "TrueCourceOkiharaModeCSForKomichiRobot.h"
#include "TrueCourceOkiharaModeCSUseRegionalValueForKomichiRobot.h" // CS完走用プログラム。地区大会の値を流用。保険として用意した安定版（不安定）
#include "TrueCourceOkiharaModeCSStableValueForOkiharaRobot.h"
#include "TrueCourceOkiharaModeCSForOkiharaRobot.h"
#include "TrueCourceSanekataModeSimulator.h" //TrueCourceSanekataCSをもとに調節したシミュレータ完走用プログラム。
#include "GoalSanekataScenarioTestMode.h" // 部分シナリオ実方コード
#include "PIDFacingAngleAbsTestMode.h" // PIDFacingAngleAbsをテストするモード
#include "GarageKomichiMode1.h" // ガレージのブロック搬入をするプログラム
#include "RGBRawReaderMode.h" // RGBRawReaderModeの場合のcommandExecutor初期化処理 // RGBRawの値をはかるプログラム。試走会用
#include "BrightnessReaderMode.h" // 明るさを取得し続けるプログラム。試走会用
// initializeCommandExecutorの定義読み込みここまで

using namespace std;
using namespace ev3api;

CommandExecutor *commandExecutor;
RobotAPI *robotAPI;

void stp_cyc_all()
{
  stp_cyc(RUNNER_CYC);
  stp_cyc(SING_A_SONG_CYC);
  stp_cyc(DEDEDON_CYC);
  stp_cyc(RETURN_TO_START_POINT_CYC);
  stp_cyc(LISTEN_BLUETOOTH_COMMAND_CYC);
}

void runner_task(intptr_t exinf)
{
  ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示
#ifdef StopWhenThrowException
  try
  {
#endif
    commandExecutor->run(); // 走らせる
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif
  ext_tsk();
}

enum ReturnToStartPointState
{
  RTSP_TURNNING_UP,
  RTSP_WALKING_UP,
  RTSP_TURNNING_RIGHT,
  RTSP_WALKING_RIGHT,
  RTSP_FINISH,
};

ReturnToStartPointState returnToStartPointState = RTSP_TURNNING_UP;
Walker *returnToStartPointStraightWalker = new Walker(20, 20);
FacingAngleAbs *facing180 = new FacingAngleAbs(FA_WheelCount, 10, 180);
FacingAngleAbs *facing90 = new FacingAngleAbs(FA_WheelCount, 10, 270);
Predicate *facing180Predicate = new FinishedCommandPredicate(facing180);
Predicate *facing90Predicate = new FinishedCommandPredicate(facing90);
bool initedFacing180 = false;
bool initedFacing90 = false;
colorid_t returnToStartPointEdgeLineColor = COLOR_RED;
bool startedBackToTheFuture = false;

void return_to_start_point_task(intptr_t exinf)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    switch (returnToStartPointState)
    {
    case RTSP_TURNNING_UP:
    {
      if (!initedFacing180)
      {
        StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
        startDededon->run(robotAPI);
        delete startDededon;

        facing180->preparation(robotAPI);
        facing180Predicate->preparation(robotAPI);
        initedFacing180 = true;
        ext_tsk();
        return;
      }
      if (!facing180Predicate->test(robotAPI))
      {
        facing180->run(robotAPI);
        ext_tsk();
        return;
      }
      else
      {
        returnToStartPointState = RTSP_WALKING_UP;
        ext_tsk();
        return;
      }
    }
    break;

    case RTSP_WALKING_UP:
    {
      returnToStartPointStraightWalker->run(robotAPI);
      colorid_t colorID = robotAPI->getColorSensor()->getColorNumber();
      if (colorID == returnToStartPointEdgeLineColor)
      {
        returnToStartPointState = RTSP_TURNNING_RIGHT;
        ext_tsk();
        return;
      }
    }
    break;

    case RTSP_TURNNING_RIGHT:
    {
      if (!initedFacing90)
      {
        facing90->preparation(robotAPI);
        facing90Predicate->preparation(robotAPI);
        initedFacing90 = true;
        ext_tsk();
        return;
      }
      if (!facing90Predicate->test(robotAPI))
      {
        facing90->run(robotAPI);
        ext_tsk();
        return;
      }
      else
      {
        returnToStartPointState = RTSP_WALKING_RIGHT;
        ext_tsk();
        return;
      }
    }
    break;

    case RTSP_WALKING_RIGHT:
    {
      returnToStartPointStraightWalker->run(robotAPI);
      ext_tsk();
      return;
    }
    break;

    case RTSP_FINISH:
    {
      // 別になくてもいっか。スタート地点に帰ってくればいいんだからな。
    }
    break;

    default:
      break;
    }

#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif
  ext_tsk();
}

enum BTCommand
{
  BTC_EMERGENCY_STOP = 's',
  BTC_RETURN_TO_START_POINT = 'r',
  BTC_NEXT_COMMAND = 'n',
};

void emergencyStop()
{
  stp_cyc_all();
  commandExecutor->emergencyStop();
}

void listen_bluetooth_command_task(intptr_t exinf)
{
#ifdef EnableBluetooth

#ifdef StopWhenThrowException
  try
  {
#endif

    unsigned char bluetoothCommand = fgetc(bt);
    switch (bluetoothCommand)
    {
    case BTC_EMERGENCY_STOP:
    {
      emergencyStop();
      Stopper *stopper = new Stopper();
      stopper->run(robotAPI);
      delete stopper;
      break;
    }
    case BTC_RETURN_TO_START_POINT:
    {
      if (!startedBackToTheFuture)
      {
        startedBackToTheFuture = true;

        const uint32_t sleepDuration = 100 * 1000;

        emergencyStop();

        vector<string> messageLines;
        messageLines.push_back("STARTED Back to the future");
        PrintMessage *printMessage = new PrintMessage(messageLines, true);
        printMessage->run(robotAPI);
        robotAPI->getClock()->sleep(sleepDuration);
        delete printMessage;

        StartCyc *startBackToTheFuture = new StartCyc(RETURN_TO_START_POINT_CYC);
        startBackToTheFuture->run(robotAPI);
        delete startBackToTheFuture;
        break;
      }
    }
    case BTC_NEXT_COMMAND:
    {
      commandExecutor->nextCommand();
      break;
    }
    default:
      break;
    }
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif
#endif

  ext_tsk();
};

CommandExecutor *singASongCommandExecutor;
void sing_a_song_task(intptr_t exinf)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    singASongCommandExecutor->run();
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif

  ext_tsk();
};

vector<Note *> dededon = generateDededon();
CommandExecutor *dededonCommandExecutor;
void dededon_task(intptr_t exinf)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    dededonCommandExecutor->run();
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }
#endif

  ext_tsk();
};

void initDededon()
{
  for (int i = 0; i < ((int)dededon.size()); i++)
  {
    dededonCommandExecutor->addCommand(dededon[i], new FinishedCommandPredicate(dededon[i]), "");
  }
};

void initSong(int loop)
{
  for (int j = 0; j < loop; j++)
  {
    for (int i = 0; i < ((int)song.size()); i++)
    {
      singASongCommandExecutor->addCommand(song[i], new FinishedCommandPredicate(song[i]), "");
    }
  }
};

void main_task(intptr_t unused)
{
#ifdef StopWhenThrowException
  try
  {
#endif
    const uint32_t sleepDuration = 100 * 1000;
    const int8_t line_height = 20;

// Bluetoothファイルを開く
#ifdef EnableBluetooth
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
#endif

    // 初期化中メッセージの出力
    int line = 1;
    ev3_lcd_fill_rect(0, line * line_height, EV3_LCD_WIDTH, line_height, EV3_LCD_WHITE);
    ev3_lcd_draw_string("**** yamatea ****", 0, 0); // 0行目の表示
    ev3_lcd_draw_string("initializing...", 0, line * line_height);

    // すべての周期ハンドラを止める
    // stp_cyc_all();

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

    // バッテリーが少なかったら音で通知する
    if (lowBatteryVoltageMv >= ev3_battery_voltage_mV())
    {
      StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
      startDededon->run(robotAPI);
      delete startDededon;
    }

    // commandExecutorを初期化する（挙動定義）
    initializeCommandExecutor(commandExecutor, robotAPI);
#ifdef Right
    commandExecutor->reverseCommandAndPredicate();
#endif

    // デデドン！
    dededonCommandExecutor = new CommandExecutor(robotAPI, false);
    initDededon();

// FroggySongを歌うCommandExecutorを初期化する
#ifdef SingASong
    singASongCommandExecutor = new CommandExecutor(robotAPI, false);
    initSong(loopSong);
    commandExecutor->addCommand(new StartCyc(SING_A_SONG_CYC), new NumberOfTimesPredicate(1), "sing a song");
#endif

    vector<string> readyMessageLines;
    readyMessageLines.push_back("ready");
    PrintMessage *printReadyMessage = new PrintMessage(resetedMessageLines, true);
    printReadyMessage->run(robotAPI);
    delete printReadyMessage;

    // commandExecutor->run()の周期ハンドラを起動する
    StartCyc *startRunnerCyc = new StartCyc(RUNNER_CYC);
    startRunnerCyc->run(robotAPI);
    delete startRunnerCyc;

#ifdef EnableBluetooth
    // bluetoothCommandを受け取る周期ハンドラを起動する
    StartCyc *startListenBluetoothCyc = new StartCyc(LISTEN_BLUETOOTH_COMMAND_CYC);
    startListenBluetoothCyc->run(robotAPI);
    delete startListenBluetoothCyc;
#endif

    // 終了判定処理
    for (; true; clock->sleep(sleepDuration))
    {
      // 左ボタンが押されたら緊急停止のためにループを抜ける
      if (ev3_button_is_pressed(LEFT_BUTTON))
      {
        // 停止処理
        commandExecutor->emergencyStop();
        dededonCommandExecutor->emergencyStop();
#ifdef SingASong
        singASongCommandExecutor->emergencyStop();
#endif
        break;
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

#ifdef EnableBluetooth
    // LISTEN_BLUETOOTH_COMMAND_CYCが走っていたら止める。
    // T_RCYC btcCycState;
    // ref_cyc(LISTEN_BLUETOOTH_COMMAND_CYC, &btcCycState);
    // if (btcCycState.cycstat == TCYC_STA)
    // {
    stp_cyc(LISTEN_BLUETOOTH_COMMAND_CYC);
    // }
#endif

#ifdef SingASong
    // 歌ってたら止める
    // T_RCYC singASongCycState;
    // ref_cyc(RETURN_TO_START_POINT_CYC, &singASongCycState);
    // if (singASongCycState.cycstat == TCYC_STA)
    // {
    stp_cyc(SING_A_SONG_CYC);
    // }
#endif

    // returnToStartPointの終了待機
    robotAPI->getClock()->sleep(sleepDuration);
    for (; true; clock->sleep(sleepDuration))
    {
      // 左ボタンが押されたら周期ハンドラを止める
      if (ev3_button_is_pressed(LEFT_BUTTON))
      {
        // 停止処理
        stp_cyc(BTC_RETURN_TO_START_POINT);
      }

      // RETURN_TO_START_POINT_CYCが終了していたら走行完了なのでループを抜ける
      T_RCYC returnToStartPointCycState;
      ref_cyc(BTC_RETURN_TO_START_POINT, &returnToStartPointCycState);
      if (returnToStartPointCycState.cycstat == TCYC_STP)
      {
        break;
      }
    }

    stp_cyc_all();

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
    delete returnToStartPointStraightWalker;
    delete facing90;
    delete facing180;
#ifdef StopWhenThrowException
  }
  catch (exception e)
  {
    stp_cyc_all();
    StartCyc *startDededon = new StartCyc(DEDEDON_CYC);
    startDededon->run(robotAPI);
    delete startDededon;

    vector<string> messageLines;
    messageLines.push_back(e.what());
    PrintMessage *printMessage = new PrintMessage(messageLines, true);
    printMessage->run(robotAPI);
    delete printMessage;

    Stopper *stopper = new Stopper();
    stopper->run(robotAPI);
    delete stopper;
  }

#endif
#ifdef EnableBluetooth
  fclose(bt);
#endif
};