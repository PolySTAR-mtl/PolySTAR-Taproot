#include "play_song_command.hpp"

#include "tap/architecture/clock.hpp"

namespace control::buzzer
{

PlaySongCommand::PlaySongCommand(BuzzerSubsystem* const buzzer, const Song& song)
    : tap::control::Command{},
      buzzer_{buzzer},
      song_{song}
{
    if (buzzer == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(buzzer));
}

PlaySongCommand::~PlaySongCommand() = default;

void PlaySongCommand::initialize() { buzzer_->playSong(song_); }

void PlaySongCommand::execute() { buzzer_->refresh(); }

void PlaySongCommand::end(bool interrupted) { buzzer_->stop(); }

bool PlaySongCommand::isFinished() const { return buzzer_->isFinished(); }

const char* PlaySongCommand::getName() const { return NAME; }

}  // namespace control::buzzer