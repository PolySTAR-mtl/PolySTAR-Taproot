#ifndef PLAY_SONG_COMMAND_HPP
#define PLAY_SONG_COMMAND_HPP


#include "tap/control/command.hpp"

#include "subsystems/buzzer/classes/song.hpp"
#include "subsystems/buzzer/buzzer_subsystem.hpp"

namespace control::buzzer
{

class PlaySongCommand : public tap::control::Command
{
public:
    PlaySongCommand(BuzzerSubsystem* const buzzer, Song song);
    ~PlaySongCommand() override;

    PlaySongCommand(const PlaySongCommand &other) = delete;
    PlaySongCommand &operator=(const PlaySongCommand &other) = delete;

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    const char* getName() const override;

    void pause();
    void resume();

private:
    using TimePointInMs = uint32_t;
    static inline constexpr const char* NAME = "play song command";
    BuzzerSubsystem* const buzzer_;
    Song song_;
    size_t playIndex_;
    TimePointInMs startTime_;
    bool isSongPlaying_;
    bool paused_;
};

}  // namespace control::buzzer


#endif