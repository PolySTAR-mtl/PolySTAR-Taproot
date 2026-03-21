#ifndef BUZZER_SUBSYSTEM_HPP
#define BUZZER_SUBSYSTEM_HPP

#include "tap/control/subsystem.hpp"

#include "subsystems/buzzer/classes/note.hpp"

namespace control::buzzer
{

class BuzzerSubsystem : public tap::control::Subsystem
{
public:
    BuzzerSubsystem(tap::Drivers *drivers);
    ~BuzzerSubsystem() override;

    BuzzerSubsystem(const BuzzerSubsystem &other) = delete;

    BuzzerSubsystem &operator=(const BuzzerSubsystem &other) = delete;

    void initialize() override;

    void refresh() override;

    void playNote(const Note& note);

    void silence();
};

}  // namespace control::buzzer

#endif  // BUZZER_SUBSYSTEM_HPP