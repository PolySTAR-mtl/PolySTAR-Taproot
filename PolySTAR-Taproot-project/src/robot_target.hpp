#ifndef ROBOT_TARGET_HPP
#define ROBOT_TARGET_HPP

namespace target {

enum class RobotTarget 
{
    Engineer,
    Hero,
    Icra,
    Sentry,
    SpinToWin,
    Standard,
};

constexpr RobotTarget ROBOT_TARGET = 
#if defined(TARGET_ENGINEER)
    RobotTarget::Engineer;
#elif defined(TARGET_HERO)
    RobotTarget::Hero;
#elif defined(TARGET_ICRA)
    RobotTarget::Icra;
#elif defined(TARGET_SENTRY)
    RobotTarget::SpinToWin;
#elif defined(TARGET_SPIN_TO_WIN)
    RobotTarget::SpinToWin;
#elif defined(TARGET_STANDARD)
    RobotTarget::Standard;
#else
    RobotTarget::Standard;
    #warning "No target defined, defaulting to standard. Define a target in the build system to remove this warning."
#endif

}

#endif // ROBOT_TARGET_HPP