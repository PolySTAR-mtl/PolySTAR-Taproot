#ifndef ROBOT_TARGET_HPP
#define ROBOT_TARGET_HPP

namespace target {

enum class RobotTarget 
{
    Standard,
    Hero,
    Sentry,
    SpinToWin,
    Engineer,
};

constexpr RobotTarget ROBOT_TARGET = 
#ifdef TARGET_STANDARD
        RobotTarget::Standard;
#elif defined(TARGET_HERO)
        RobotTarget::Hero;
#elif defined(TARGET_SENTRY)
        RobotTarget::Sentry;
#elif defined(TARGET_SPIN_TO_WIN)
        RobotTarget::SpinToWin;
#elif defined(TARGET_ENGINEER)
        RobotTarget::Engineer;
#else
        RobotTarget::Standard;
#endif

}

#endif // ROBOT_TARGET_HPP