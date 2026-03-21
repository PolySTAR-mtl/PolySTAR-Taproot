#pragma once

namespace control::chassis::algorithms
{
    struct Gains2 {
        float k_pos;
        float k_vel;
    };
}

#ifdef TARGET_HERO
// defauilt lqr gains (standard)
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_SENTRY
// defauilt lqr gains (standard)
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_STANDARD
// defauilt lqr gains (standard)
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_ICRA
// defauilt lqr gains (standard)
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif

#ifdef TARGET_SPIN_TO_WIN
// defauilt lqr gains (standard)
static constexpr control::chassis::algorithms::Gains2 Kx = {63.24f, 11.25f}; 
static constexpr control::chassis::algorithms::Gains2 Ky = {63.24f, 11.25f};
static constexpr control::chassis::algorithms::Gains2 Kt = {15.81f, 2.45f};
#endif
