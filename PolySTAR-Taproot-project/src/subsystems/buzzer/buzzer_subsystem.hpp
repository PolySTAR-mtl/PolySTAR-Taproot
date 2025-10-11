#ifndef BUZZER_SUBSYSTEM_HPP
#define BUZZER_SUBSYSTEM_HPP

#include "tap/control/subsystem.hpp"
#include "control/drivers/drivers.hpp"

namespace control
{
namespace buzzer
{

static constexpr uint16_t BASE_FREQUENCY_HZ = 440;
static constexpr uint8_t BASE_FREQUENCY_NOTE = 69; 

static constexpr uint8_t marioTheme[][2] = 
{
    {76,150}, {76,150}, {0,150}, {76,150}, {0,150}, {72,150}, {76,150}, {0,150},
    {79,300}, {0,300}, {67,300}, {0,300},
    {72,150}, {0,150}, {67,150}, {0,150}, {64,150}, {0,150}, {69,150}, {0,150},
    {71,150}, {0,150}, {70,150}, {69,150}, {0,150}, {67,200},
    {76,150}, {79,150}, {81,150}, {0,150}, {77,150}, {79,150}, {0,150}, {76,150},
    {72,150}, {74,150}, {0,150}, {71,150}, {0,150}, {72,150}, {0,150},
    {67,300}
};

class Buzzer : public tap::control::Subsystem
{
public:

    Buzzer(src::Drivers* drivers) : tap::control::Subsystem(drivers){}

    void initialize() override;

    void refresh() override;

    void setFrequency(uint16_t frequency);

    uint16_t convertNoteToFrequency(uint8_t note);
    

private:

    src::Drivers* drivers;



};
}

}

#endif //BUZZER_SUBSYSTEM_HPP