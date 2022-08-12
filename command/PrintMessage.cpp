#include "PrintMessage.h"
#include "ev3api.h"
#include "vector"
#include "string"
#include "Setting.h"

using namespace std;

PrintMessage::PrintMessage(vector<string> ml, bool fp)
{
    // 一応ディープコピーしておきます。
    vector<string> messageLinesTemp;
    for (int i = 0; i < ((int)ml.size()); i++)
    {
        string message = string(ml[i].c_str());
        messageLinesTemp.push_back(message);
    }
    messageLines = messageLinesTemp;

    forcePrint = fp;
}

void PrintMessage::print()
{
    int i = 0;
    for (; i < ((int)messageLines.size()); i++)
    {
        string messageLine = messageLines[i];
        string messageLineEOLAppended = "";

        if (enablePrintMessageForConsole || enablePrintMessageForBluetooth)
        {
            messageLineEOLAppended.append(messageLines[i]);
            messageLineEOLAppended.append(EOL_STR);
        }

        // 出力処理
        msg_f(messageLine, i + 1);

        if (enablePrintMessageForConsole)
        {
            printf("%s", messageLineEOLAppended.c_str());
        }

        if (enablePrintMessageForBluetooth)
        {
            msg_bt(messageLineEOLAppended);
        }
    }

    // 下の行の上書き処理。
    // 前のメッセージ行数が多いときにこの処理を挟まないと、前のメッセージが残り続ける。
    for (; i < 6; i++)
    {
        msg_f("", i + 1);
    }
}

void PrintMessage::run()
{
    if (forcePrint)
    {
        print();
    }
    else
    {
        if (enablePrintMessageMode)
        {
            print();
        }
    }
}

PrintMessage *PrintMessage::generateReverseCommand()
{
    return new PrintMessage(messageLines, forcePrint);
}

/**
 * 行単位で引数の文字列を表示
 * @param str 表示する文字列
 * @param line 20ドットごとの行番号（1から5）
 */
void PrintMessage::msg_f(string str, int32_t line)
{
    const int8_t line_height = 20;
    ev3_lcd_fill_rect(0, line * line_height, EV3_LCD_WIDTH, line_height, EV3_LCD_WHITE);
    ev3_lcd_draw_string(str.c_str(), 0, line * line_height);
}

#if defined(EnableBluetooth)
FILE *bt = ev3_serial_open_file(EV3_SERIAL_BT);
void PrintMessage::msg_bt(string str)
{
    fprintf(bt, str.c_str());
}
#else
void PrintMessage::msg_bt(string str)
{
}
#endif
