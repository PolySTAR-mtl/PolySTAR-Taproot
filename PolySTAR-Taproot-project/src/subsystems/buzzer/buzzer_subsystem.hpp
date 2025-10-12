#ifndef BUZZER_SUBSYSTEM_HPP
#define BUZZER_SUBSYSTEM_HPP

#include "tap/control/subsystem.hpp"
#include "control/drivers/drivers.hpp"
#include "tap/util_macros.hpp"
#include "tap/architecture/timeout.hpp"

namespace control
{
namespace buzzer
{

static constexpr uint16_t BASE_FREQUENCY_HZ = 440;
static constexpr uint8_t BASE_FREQUENCY_NOTE = 69; 
static constexpr uint8_t N_HALF_TONE_PER_OCTAVE = 12;

/**
 * This subsystem can play anytype of sound for a specified amount of time.
 */
class BuzzerSubsystem : public tap::control::Subsystem
{
public:

    BuzzerSubsystem(src::Drivers* drivers) : tap::control::Subsystem(drivers){}
    DISALLOW_COPY_AND_ASSIGN(BuzzerSubsystem);

    /**
     * Plays a MIDI note for a specified amount of time.
     * This method will take the note given in input, tranform it
     * into a frequency and then play it for said amount of time.
     * @param [in] midiNote 
     * @param [in] duration
     * @return Nothing.
     */
    void playNote(uint8_t midiNote);

    /**
     * Stops playing whatever it's playing.
     */
    void stopSound();
    

private:

    /**
     * Converts a midi note into frequency in hz.
     * @param [in] midiNote
     * @return The fequence that corresponds to the midi note.
     */
    uint32_t convertMidiNoteIntoFrequency(uint8_t midiNote);

    src::Drivers* drivers;
};
}

}

#endif //BUZZER_SUBSYSTEM_HPP