namespace turret::algorithms
{
    struct Gains2 {
        float k_pos;
        float k_vel;
    };
}

#ifdef TARGET_HERO
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 3.59f};
#endif

#ifdef TARGET_SENTRY
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.62f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 1.92f};
#endif

#ifdef TARGET_STANDARD
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 1.92f};
#endif

#ifdef TARGET_ICRA
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 3.59f};
#endif

#ifdef TARGET_SPIN_TO_WIN
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 1.92f};
#endif