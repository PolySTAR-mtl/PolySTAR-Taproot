#pragma once

namespace control::chassis::algorithms
{
    struct Gains2 {
        float k_pos;
        float k_vel;
    };
}

#ifdef TARGET_HERO
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_SENTRY
static constexpr control::chassis::algorithms::Gains2 Kx = {20.00f, 20.00f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {20.00f, 20.00f};
static constexpr control::chassis::algorithms::Gains2 Kt = {10.00f,  4.00f};
#endif

#ifdef TARGET_STANDARD
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_ICRA
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_SPIN_TO_WIN
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif
