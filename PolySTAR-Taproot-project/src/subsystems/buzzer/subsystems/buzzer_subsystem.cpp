#include "buzzer_subsystem.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/drivers.hpp"

namespace control::buzzer
{

BuzzerSubsystem::BuzzerSubsystem(tap::Drivers* drivers)
    : tap::control::Subsystem{drivers},
      playIndex_{},
      startTime_{},
      isPlaying_{},
      paused_{},
      song_{}
{
}

BuzzerSubsystem::~BuzzerSubsystem() = default;

void BuzzerSubsystem::initialize() { stop(); }

void BuzzerSubsystem::refresh()
{
    if (!isPlaying_ || paused_ || playIndex_ >= song_.size())
    {
        return;
    }

    const TimePointInMs currentTime = tap::arch::clock::getTimeMilliseconds();

    if (currentTime - startTime_ < song_[playIndex_].durationMs)
    {
        return;
    }

    if (++playIndex_ >= song_.size())
    {
        stop();
        return;
    }

    playNote(song_[playIndex_]);
    startTime_ = currentTime;
}

void BuzzerSubsystem::playSong(const Song& song)
{
    if (song.empty())
    {
        return;
    }

    song_ = song;
    playIndex_ = 0;
    paused_ = false;
    isPlaying_ = true;

    playNote(song_[playIndex_]);
    startTime_ = tap::arch::clock::getTimeMilliseconds();
}

void BuzzerSubsystem::pause()
{
    if (!isPlaying_ || paused_) return;

    paused_ = true;
    silence();
}

void BuzzerSubsystem::stop()
{
    silence();
    isPlaying_ = false;
    paused_ = false;
    song_ = Song{};
    playIndex_ = 0;
}

void BuzzerSubsystem::resume()
{
    if (!paused_ || !isPlaying_ || playIndex_ >= song_.size())
    {
        return;
    }

    paused_ = false;
    playNote(song_[playIndex_]);
    startTime_ = tap::arch::clock::getTimeMilliseconds();
}

void BuzzerSubsystem::playNote(const Note& note)
{
    if (note.frequency == 0)
    {
        silence();
        return;
    }
    tap::buzzer::playNote(&drivers->pwm, note.frequency);
}

[[nodiscard]] bool BuzzerSubsystem::isFinished() const { return !isPlaying_; }

void BuzzerSubsystem::silence() { tap::buzzer::silenceBuzzer(&drivers->pwm); }

}  // namespace control::buzzer
