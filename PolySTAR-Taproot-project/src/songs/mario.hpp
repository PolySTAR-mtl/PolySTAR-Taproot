#ifndef MARIO_HPP
#define MARIO_HPP

#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace songs
{

void playMarioThemesongBlocking(tap::gpio::Pwm *pwmController);

}

#endif  // MARIO_HPP