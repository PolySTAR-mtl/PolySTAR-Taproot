#ifndef BUZZER_CONSTANTS_HPP
#define BUZZER_CONSTANTS_HPP

#include <cstdint>

static constexpr uint16_t BASE_FREQUENCY_HZ = 440;
static constexpr uint8_t BASE_FREQUENCY_NOTE = 69;
static constexpr uint8_t N_HALF_TONE_PER_OCTAVE = 12;

uint16_t SONG_MARIO_THEME[][2] = {
    {76, 150}, {76, 150}, {0, 150},  {76, 150}, {0, 150},  {72, 150},
    {76, 150}, {0, 150},  {79, 300}, {0, 300},  {67, 300}, {0, 300},
    {72, 150}, {0, 150},  {67, 150}, {0, 150},  {64, 150}, {0, 150},
    {69, 150}, {0, 150},  {71, 150}, {0, 150},  {70, 150}, {69, 150},
    {0, 150},  {67, 200}, {76, 150}, {79, 150}, {81, 150}, {0, 150},
    {77, 150}, {79, 150}, {0, 150},  {76, 150}, {72, 150}, {74, 150},
    {0, 150},  {71, 150}, {0, 150},  {72, 150}, {0, 150},  {67, 300}};

uint8_t themeSize = sizeof(SONG_MARIO_THEME) / sizeof(SONG_MARIO_THEME[0]);

#endif  // BUZZER_CONSTANTS_HPP
