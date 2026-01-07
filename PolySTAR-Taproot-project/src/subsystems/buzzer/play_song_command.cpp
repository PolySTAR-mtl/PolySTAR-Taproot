#include "play_song_command.hpp"

namespace control {
namespace buzzer {
PlaySongCommand::PlaySongCommand(BuzzerSubsystem *const buzzer, src::Drivers *drivers, const uint16_t (*notes)[2], uint8_t songLength)
    : buzzer(buzzer),
      drivers(drivers),
      notes(notes),
      songLength(songLength),
      readIndex(0),
      songFinished(false) {
    if (buzzer == nullptr) {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(buzzer));
}

void PlaySongCommand::initialize() { delayTimer.restart(0); }

void PlaySongCommand::execute() {
    if (readIndex >= songLength) {
        songFinished = true;
    }
    if (delayTimer.execute()) {
        delayTimer.restart(notes[readIndex][NOTE_DURATION_INDEX]);
        buzzer->playNote(notes[readIndex][MIDI_NOTE_INDEX]);
        readIndex++;
    }
}

bool PlaySongCommand::isFinished() const { return songFinished; }

void PlaySongCommand::end(bool) { buzzer->stopSound(); }

const char *PlaySongCommand::getName() const { return "buzzer command"; }

}  // namespace buzzer
}  // namespace control
