#include "buzzer_subsystem.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/drivers.hpp"

namespace control::buzzer
{

BuzzerSubsystem::BuzzerSubsystem(tap::Drivers *drivers) 
: tap::control::Subsystem{drivers}
{   
}

BuzzerSubsystem::~BuzzerSubsystem() = default;

void BuzzerSubsystem::initialize()  {

}

void BuzzerSubsystem::refresh() {

}

void BuzzerSubsystem::playNote(const Note& note) {
    tap::buzzer::playNote(&drivers->pwm, note.frequency);
}

void BuzzerSubsystem::silence() {
    tap::buzzer::silenceBuzzer(&drivers->pwm);
}

}
