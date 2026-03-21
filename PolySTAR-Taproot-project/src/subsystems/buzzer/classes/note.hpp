#ifndef NOTE_HPP 
#define NOTE_HPP

#include <cstdint>

namespace control::buzzer
{

struct Note
{
    uint32_t frequency;
    uint32_t durationMs;
};

}  // namespace control::buzzer

#endif // NOTE_HPP