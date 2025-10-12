

#include "buzzer_subsystem.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "modm/architecture/interface/delay.hpp"

namespace control
{
namespace buzzer
{

    void BuzzerSubsystem::playNote(uint8_t midiNote, uint16_t duration)
    {
        // Converts the note:
        uint32_t convertedFrequency = convertMidiNoteIntoFrequency(midiNote);

        // Plays it for said amount of time :
        tap::buzzer::playNote(&drivers->pwm, convertedFrequency);
        modm::delay_ms(static_cast<uint32_t>(duration));
    }

    uint32_t BuzzerSubsystem::convertMidiNoteIntoFrequency(uint8_t midiNote)
    {
        double exponant = static_cast<double>(midiNote - BASE_FREQUENCY_NOTE) / N_HALF_TONE_PER_OCTAVE;
        double correspondingFrequency = BASE_FREQUENCY_HZ * (std::pow(2.0, exponant));

        return static_cast<uint32_t>(correspondingFrequency);
    }

    void BuzzerSubsystem::stopSound()
    {
        tap::buzzer::silenceBuzzer(&drivers->pwm);
    }

}
}