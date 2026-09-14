#ifndef ROBOT_TARGET_HPP
#define ROBOT_TARGET_HPP

namespace target {

enum class RobotTarget
{
    Engineer,
    Hero,
    Sentry,
    Standard,
};

constexpr RobotTarget ROBOT_TARGET =
#if defined(TARGET_ENGINEER)
    RobotTarget::Engineer;
#elif defined(TARGET_STANDARD)
    RobotTarget::Standard;
#elif defined(TARGET_HERO)
    RobotTarget::Hero;
#elif defined(TARGET_SENTRY)
    RobotTarget::Sentry;
#else
    RobotTarget::Standard;
    #warning "No target defined, defaulting to standard. Define a target in the build system to remove this warning."
#endif

}

#endif // ROBOT_TARGET_HPP