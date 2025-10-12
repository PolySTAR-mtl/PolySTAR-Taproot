#ifndef BUZZER_COMMAND_HPP_
#define BUZZER_COMMAND_HPP_

#include "buzzer_subsystem.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/command.hpp"
#include "tap/util_macros.hpp"

namespace control
{
namespace buzzer
{

static constexpr uint8_t NOTE_DURATION_INDEX = 1;
static constexpr uint8_t MIDI_NOTE_INDEX = 0;

class BuzzerCommand : public tap::control::Command
{
public:
    BuzzerCommand(
        BuzzerSubsystem *const buzzer, 
        src::Drivers *drivers,
        uint16_t (*notes)[2],
        uint8_t songLength,
        uint8_t readIndex = 0);
    // BuzzerCommand(const BuzzerSubsystem &other ) = delete;
    // BuzzerCommand &operator=(const BuzzerSubsystem &other ) = delete;
    DISALLOW_COPY_AND_ASSIGN(BuzzerCommand);
    
    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;
    const char* getName() const override;

private:

    void setSongLenght(uint16_t** notes);
    BuzzerSubsystem *const buzzer;   
    src::Drivers *drivers;
    tap::arch::MilliTimeout delayTimer;
    uint16_t (*notes)[2];
    uint8_t songLength;
    uint8_t readIndex;
    bool songFinished;
};
}

}

#endif //BUZZER_COMMAND_HPP