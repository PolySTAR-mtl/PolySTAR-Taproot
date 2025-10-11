

#include "buzzer_subsystem.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace control
{
namespace buzzer
{
    void Buzzer::initialize() 
    {

    }

    void Buzzer::refresh() 
    {

    }

    void Buzzer::setFrequency(uint16_t frequency)
    {
        tap::buzzer::playNote(&drivers->pwm, frequency);
    }

    uint16_t Buzzer::convertNoteToFrequency(uint8_t note)
    {
        int16_t halfToneDifference = note - BASE_FREQUENCY_NOTE;
        
        uint32_t convertedFrequency = BASE_FREQUENCY_HZ * (std::pow(2, halfToneDifference));
        return convertedFrequency;
    }

}
}