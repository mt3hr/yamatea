#include "util.h"
#include "string"

using namespace std;

// 演習用のユーティリティ

// 初期処理用
void init_f(string str)
{
  // フォントの設定と0行目の表示
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string(str.c_str(), 0, 0);
}

/**
 * 行単位で引数の文字列を表示
 * @param str 表示する文字列
 * @param line 20ドットごとの行番号（1から5）
 */
void msg_f(string str, int32_t line)
{
  const int8_t line_height = 20;
  ev3_lcd_fill_rect(0, line * line_height, EV3_LCD_WIDTH, line_height, EV3_LCD_WHITE);
  ev3_lcd_draw_string(str.c_str(), 0, line * line_height);
}

#if defined(EnableBluetooth)
FILE *bt = ev3_serial_open_file(EV3_SERIAL_BT);
void msg_bt(string str)
{
  fprintf(bt, str.c_str());
}
#endif