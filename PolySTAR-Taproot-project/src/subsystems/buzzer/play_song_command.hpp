#ifndef PLAY_SONG_COMMAND_HPP_
#define PLAY_SONG_COMMAND_HPP_

#include "buzzer_subsystem.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/command.hpp"
#include "tap/util_macros.hpp"

namespace control {
namespace buzzer {

static constexpr uint8_t NOTE_DURATION_INDEX = 1;
static constexpr uint8_t MIDI_NOTE_INDEX = 0;

class PlaySongCommand : public tap::control::Command {
   public:
    PlaySongCommand(BuzzerSubsystem *const buzzer, src::Drivers *drivers, const uint16_t (*notes)[2], uint8_t songLength);

    PlaySongCommand(const PlaySongCommand &) = delete;
    PlaySongCommand &operator=(const PlaySongCommand &) = delete;

    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;
    const char *getName() const override;

   private:
    BuzzerSubsystem *const buzzer;
    src::Drivers *drivers;
    tap::arch::MilliTimeout delayTimer;
    const uint16_t (*notes)[2];
    uint8_t songLength;
    uint8_t readIndex;
    bool songFinished;

};  // class PlaySongCommand

}  // namespace buzzer

}  // namespace control

#endif  // PLAY_SONG_COMMAND_HPP_