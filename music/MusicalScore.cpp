#include "MusicalScore.h"
#include "vector"

using namespace std;

vector<Note *> generateFroggySong()
{
    int duration = 250;
    int volume = 30;

    Note *froggySongNotes[] = {
        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_D4, duration, volume),
        new Note(NOTE_E4, duration, volume),
        new Note(NOTE_F4, duration, volume),
        new Note(NOTE_E4, duration, volume),
        new Note(NOTE_D4, duration, volume),
        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_C4, duration, 0),

        new Note(NOTE_E4, duration, volume),
        new Note(NOTE_F4, duration, volume),
        new Note(NOTE_G4, duration, volume),
        new Note(NOTE_A5, duration, volume),
        new Note(NOTE_G4, duration, volume),
        new Note(NOTE_F4, duration, volume),
        new Note(NOTE_E4, duration, volume),
        new Note(NOTE_C4, duration, 0),

        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_C4, duration, 0),

        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_C4, duration, 0),

        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_C4, duration, 0),

        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_C4, duration, 0),

        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_D4, duration, volume),
        new Note(NOTE_E4, duration, volume),
        new Note(NOTE_F4, duration, volume),
        new Note(NOTE_E4, duration, volume),
        new Note(NOTE_D4, duration, volume),
        new Note(NOTE_C4, duration, volume),
        new Note(NOTE_C4, duration, 0),
    };

    vector<Note *> result;
    for (int i = 0; i < ((int)(sizeof(froggySongNotes) / sizeof(froggySongNotes[0]))); i++)
    {
        result.push_back(froggySongNotes[i]);
    }
    return result;
};
