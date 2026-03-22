#ifndef PLAY_SONG_COMMAND_HPP
#define PLAY_SONG_COMMAND_HPP

#include "tap/control/command.hpp"

#include "subsystems/buzzer/classes/song.hpp"
#include "subsystems/buzzer/subsystems/buzzer_subsystem.hpp"

namespace control::buzzer
{

class PlaySongCommand : public tap::control::Command
{
public:
    PlaySongCommand(BuzzerSubsystem* const buzzer, const Song& song);
    ~PlaySongCommand() override;

    PlaySongCommand(const PlaySongCommand& other) = delete;
    PlaySongCommand& operator=(const PlaySongCommand& other) = delete;

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    const char* getName() const override;

private:
    static inline constexpr const char* NAME = "play song command";
    BuzzerSubsystem* const buzzer_;
    Song song_;
};

}  // namespace control::buzzer

#endif