#ifndef PrintMessage_H
#define PrintMessage_H

#include "Command.h"
#include "string"
#include "vector"
#include "ev3api.h"

using namespace std;

// PrintMessage
// メッセージをディスプレイやコンソール、Bluetooth端末に出力するコマンド
// 出力先設定はapp.cppの enablePrintMessageMode, enablePrintMessageForConsole, enablePrintMessageForBluetoothから指定できます。
// 
// 実方
class PrintMessage : public Command
{
private:
    const string EOL_STR = string("\r\n");
    vector<string> messageLines;
    bool forcePrint;
    void print();
    void msg_f(string str, int32_t line);
    void msg_bt(string str);

public:
    PrintMessage(vector<string> messageLines, bool forcePrint);
    void run() override;
    PrintMessage *generateReverseCommand() override;
};

#endif