#ifndef MY_UNTL_H_
#define MY_UNTL_H_

#include "ev3api.h"
#include "string"

using namespace std;

extern void init_f(string str);
extern void msg_f(string str, int32_t line);
extern void msg_bt(string str);

#endif
