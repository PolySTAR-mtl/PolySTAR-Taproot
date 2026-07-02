namespace turret::algorithms
{
    struct Gains2 {
        float k_pos;
        float k_vel;
    };
}

#ifdef TARGET_HERO
static constexpr turret::algorithms::Gains2 Kyaw  = {65.00f, 8.00f}; //45, 3
static constexpr turret::algorithms::Gains2 Kpitch = {105.00f, 4.15f}; // 65, 3
#endif

#ifdef TARGET_SENTRY
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 1.92f};
#endif

#ifdef TARGET_STANDARD
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {45.00f, 2.50f};
#endif

#ifdef TARGET_ICRA
static constexpr turret::algorithms::Gains2 Kyaw  = {31.62f, 3.46f};
static constexpr turret::algorithms::Gains2 Kpitch = {31.62f, 3.59f};
#endif

#ifdef TARGET_SPIN_TO_WIN
static constexpr turret::algorithms::Gains2 Kyaw  = {45.00f, 4.10};
static constexpr turret::algorithms::Gains2 Kpitch = {100.00f, 5.00f};
#endif