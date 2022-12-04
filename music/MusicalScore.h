#ifndef MusicalScore_H
#define MusicalScore_H

#include "Note.h"
#include "ev3api.h"
#include "vector"
#include "Setting.h"

using namespace std;

// 曲。
// 実方

// カエルの歌。実方
vector<Note *> generateFroggySong();

// デデドン！（絶望）。実方
vector<Note *> generateDededon();

// FreedomDive↓
vector<Note *> generateFreedomDive();

#endif