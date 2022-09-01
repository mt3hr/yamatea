#include "MusicalScore.h"
#include "vector"

using namespace std;

vector<Note *> generateFroggySong()
{
    Note *froggySongNotes[] = {
        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_D4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_E4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_F4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_E4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_D4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(0, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),

        new Note(NOTE_E4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_F4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_G4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_A5, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_G4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_F4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_E4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(0, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),

        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(0, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),

        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(0, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),

        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(0, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),

        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(0, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),

        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_D4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_E4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_F4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_E4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_D4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
        new Note(NOTE_C4, beepNoteWhenCommandSwitching->getDuration(), beepNoteWhenCommandSwitching->getVolume()),
    };

    vector<Note *> result(begin(froggySongNotes), end(froggySongNotes));
    return result;
};
