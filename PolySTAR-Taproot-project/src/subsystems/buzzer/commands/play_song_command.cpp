#include "play_song_command.hpp"

#include "tap/architecture/clock.hpp"

namespace control::buzzer
{

PlaySongCommand::PlaySongCommand(BuzzerSubsystem* const buzzer, Song song) : tap::control::Command{},
    buzzer_{buzzer},
    song_{song},
    playIndex_{},
    isSongPlaying_{},
    paused_{}
{
    if (buzzer == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(buzzer));

}

PlaySongCommand::~PlaySongCommand() = default;

void PlaySongCommand::initialize() {
    playIndex_ = 0;
    isSongPlaying_ = false;
    paused_ = false;
}

void PlaySongCommand::execute() {
    if (paused_ || playIndex_ >= song_.size()) {
        return;
    }

    const TimePointInMs currentTime = tap::arch::clock::getTimeMilliseconds();
    if (isSongPlaying_ && currentTime - startTime_ < song_[playIndex_].durationMs) {
        return;
    }
    startTime_ = currentTime;

    if (isSongPlaying_) {
        if (playIndex_ + 1 >= song_.size()) {
            playIndex_ = song_.size();
            return;
        }

        buzzer_->playNote(song_[++playIndex_]);
        return;
    }

    buzzer_->playNote(song_[playIndex_]);
    isSongPlaying_ = true;
}

void PlaySongCommand::end(bool interrupted) {
    buzzer_->silence();
    isSongPlaying_ = false;
}

bool PlaySongCommand::isFinished() const {
    return playIndex_ >= song_.size();
}

const char* PlaySongCommand::getName() const { return NAME; }

void PlaySongCommand::pause() {
    paused_ = true;
    buzzer_->silence();
}

void PlaySongCommand::resume() {
    paused_ = false;
    if (playIndex_ < song_.size()) {
        buzzer_->playNote(song_[playIndex_]);
        startTime_ = tap::arch::clock::getTimeMilliseconds();
        isSongPlaying_ = true;
    }
}

}