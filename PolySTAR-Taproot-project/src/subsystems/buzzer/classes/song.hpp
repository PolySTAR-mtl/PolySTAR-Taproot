#ifndef SONG_HPP
#define SONG_HPP

#include <span>

#include "subsystems/buzzer/classes/note.hpp"

namespace control::buzzer
{

using Song = std::span<const Note>;

};

#endif