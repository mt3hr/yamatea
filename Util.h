#ifndef Util_H
#define Util_H

#include "vector"
#include "string"
#include "sstream"

using namespace std;

// 文字列sを区切り文字delimで区切ってvectorで返す関数
// 実方
vector<string> split(const string &s, char delim);

#endif