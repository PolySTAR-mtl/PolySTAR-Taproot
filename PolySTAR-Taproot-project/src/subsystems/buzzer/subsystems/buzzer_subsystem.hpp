#ifndef BUZZER_SUBSYSTEM_HPP
#define BUZZER_SUBSYSTEM_HPP

#include "tap/control/subsystem.hpp"

#include "subsystems/buzzer/classes/note.hpp"
#include "subsystems/buzzer/classes/song.hpp"

namespace control::buzzer
{

class BuzzerSubsystem : public tap::control::Subsystem
{
public:
    BuzzerSubsystem(tap::Drivers* drivers);
    ~BuzzerSubsystem() override;

    BuzzerSubsystem(const BuzzerSubsystem& other) = delete;

    BuzzerSubsystem& operator=(const BuzzerSubsystem& other) = delete;

    void initialize() override;

    void refresh() override;

    void playSong(const Song& song);

    void pause();

    void resume();

    void stop();

    [[nodiscard]] bool isFinished() const;

private:
    void playNote(const Note& note);
    void silence();

private:
    using TimePointInMs = uint32_t;

private:
    size_t playIndex_;
    TimePointInMs startTime_;
    bool isPlaying_;
    bool paused_;
    Song song_;
};

}  // namespace control::buzzer

#endif  // BUZZER_SUBSYSTEM_HPP