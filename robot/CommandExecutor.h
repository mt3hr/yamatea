#ifndef CommandExecutor_H
#define CommandExecutor_H

#include "Predicate.h"
#include "Command.h"
#include <vector>
#include "string"
#include "RobotAPI.h"

using namespace std;
using namespace ev3api;

// CommandExecutor
// Commandの実行者。
// ロボット操作命令を逐次実行していくもの。
// メインタスクから呼び出される。
//
// 実方
class CommandExecutor
{
private:
    int currentIndexForCommand;
    vector<Command *> commands;
    vector<Predicate *> predicates;
    vector<string> commandNames;
    vector<bool> preparated;
    vector<bool> printedCommandName;
    RobotAPI *robotAPI;
    bool runner;
    uint64_t loopCount = 0;
    uint64_t time = 0;

public:
    CommandExecutor(RobotAPI *robotAPI, bool runner);
    virtual ~CommandExecutor();

    // addCommand
    // Commandを追加する。
    // command: 追加対象のコマンド
    // exitCondition: コマンドの終了条件
    // commandName: コマンド名（デバッグ用）
    virtual void addCommand(Command *command, Predicate *exitCondition, string commandName);

    // run
    // コマンドを実行する。周期ハンドラから呼び出される。
    virtual void run();

    // nextCommand。
    // 現在のコマンドを強制的に終了し、次のコマンドに遷移する。
    // Bluetoothでのリモートスタートを実現するためにpublicメソッドになった
    virtual void nextCommand();

    // emergencyStop
    // 暴走時用の緊急停止メソッド。
    // 左ボタンから停止できるようにpublicメソッドになった。
    virtual void emergencyStop();

    // reverseCommandAndPredicate
    // 現在追加されているCommandとPredicateを左右反転する。
    // 左右コース対応のために設けられたメソッド。
    virtual void reverseCommandAndPredicate();

    // isFinished
    // すべてのコマンドを実行し終えたらtrueを返すメソッド。
    // タスクの終了をする際に知る必要がある情報なのでpublicメソッドになった
    virtual bool isFinished();
};

#endif
