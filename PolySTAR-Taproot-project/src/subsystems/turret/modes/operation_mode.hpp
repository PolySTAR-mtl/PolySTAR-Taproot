#ifndef OPERATION_MODE_HPP_
#define OPERATION_MODE_HPP_

#include "subsystems/turret/commands/turret_sentry_command.hpp"
#include "subsystems/turret/commands/turret_hero_command.hpp"
#include "subsystems/turret/commands/turret_spin2win_command.hpp"

namespace control::turret
{

struct OperationMode
{
    // SHOULD CHANGE TO SENTRY COMMAND
    static void autoMode(SentryAimCommand* command);
    static void manualMode(HeroAimCommand* command);
    static void manualMode(Spin2WinAimCommand* command);
};

struct AutoAttributes
{
    const float MRAD_TO_DEGREES = 0.0572958;
};

struct ManualAttributes
{
    // Need to set to tap::arch::clock::getTimeMilliseconds() on command initialization
    uint32_t prevUpdate;

    uint32_t compoundedTime = 0;
    float chassisRotationSpeed = 0;
    int gzSamplingCount = 0;
    float gzSamplingSum = 0;
    float gzAverage = 0;
};

}  // namespace control::turret

#endif // OPERATION_MODE_HPP_