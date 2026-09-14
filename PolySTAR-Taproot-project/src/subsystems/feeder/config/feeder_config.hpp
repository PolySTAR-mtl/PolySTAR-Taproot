#ifndef FEEDER_CONFIG_HPP
#define FEEDER_CONFIG_HPP

#include <stdint.h>

#include "robot_target.hpp"
#include "subsystems/feeder/config/feeder_constants.hpp"
#include "tap/communication/can/can_bus.hpp"

namespace control::feeder {

struct FeederConfig {
    // Used for the velocity subsystem
    float feederRpm; // The feeder RPM when the feeder is on
    float feederReverseRpm; // The feeder RPM when the feeder is unjamming

    // Used for position subsystem
    float unjamMaxWaitTimeMs; // Maximum time to wait for unjamming
    float moveDisplacementTick; // Displacement in ticks for each move during unjamming
    float unjamCycles; // Number of cycles to perform during unjamming
    float unjamDisplacementTick; // Displacement in ticks for each unjamming move
    float pauseAfterMoveTimeMs; // Time to pause after each move during unjamming
    float moveTimeMs; // Time to perform each move during unjamming
    float setpointPosToleranceTick; // Tolerance in ticks for considering the feeder at the setpoint position

    float jamCheckerToleranceTick; // Tolerance in ticks for the jam checker
    uint32_t jamCheckerToleranceMs; // Time in milliseconds for the jam checker tolerance

    bool isFeederInverted; // Whether the feeder motor is inverted

    tap::can::CanBus canBusMotors; // CAN bus for the feeder motors
};

constexpr FeederConfig BASE_FEEDER_CONFIG {
    .feederRpm = 2500.0f,
    .feederReverseRpm = -1500.0f,

    .unjamMaxWaitTimeMs = 500.0f,
    .moveDisplacementTick = 45 * DEGREE_TO_TICK,
    .unjamCycles = 4.0f,
    .unjamDisplacementTick = 45 * DEGREE_TO_TICK,
    .pauseAfterMoveTimeMs = 100.0f,
    .moveTimeMs = 125.0f,
    .setpointPosToleranceTick = 45 * DEGREE_TO_TICK,

    .jamCheckerToleranceTick = 5 * DEGREE_TO_TICK,
    .jamCheckerToleranceMs = 500,

    .isFeederInverted = true,

    .canBusMotors = tap::can::CanBus::CAN_BUS1,
};

constexpr FeederConfig STANDARD_FEEDER_CONFIG = BASE_FEEDER_CONFIG;

constexpr FeederConfig HERO_FEEDER_CONFIG {
    .feederRpm = 2500.0f,
    .feederReverseRpm = -1500.0f,

    .unjamMaxWaitTimeMs = 500.0f,
    .moveDisplacementTick = 45 * DEGREE_TO_TICK,
    .unjamCycles = 1.0f,
    .unjamDisplacementTick = 45 * DEGREE_TO_TICK,
    .pauseAfterMoveTimeMs = 100.0f,
    .moveTimeMs = 300.0f,
    .setpointPosToleranceTick = 45 * DEGREE_TO_TICK,

    .jamCheckerToleranceTick = 20 * DEGREE_TO_TICK,
    .jamCheckerToleranceMs = 500,

    .isFeederInverted = true,

    .canBusMotors = tap::can::CanBus::CAN_BUS1,
};

constexpr FeederConfig SENTRY_FEEDER_CONFIG {
    .feederRpm = 2500.0f,
    .feederReverseRpm = -1500.0f,

    .unjamMaxWaitTimeMs = 500.0f,
    .moveDisplacementTick = -90 * DEGREE_TO_TICK,
    .unjamCycles = 4.0f,
    .unjamDisplacementTick = -45 * DEGREE_TO_TICK,
    .pauseAfterMoveTimeMs = 500.0f,
    .moveTimeMs = 200.0f,
    .setpointPosToleranceTick = 1 * DEGREE_TO_TICK,

    .jamCheckerToleranceTick = 5 * DEGREE_TO_TICK,
    .jamCheckerToleranceMs = 500,

    .isFeederInverted = false,

    .canBusMotors = tap::can::CanBus::CAN_BUS1,
};

constexpr FeederConfig SPIN_TO_WIN_FEEDER_CONFIG {
    .feederRpm = 2500.0f,
    .feederReverseRpm = -1500.0f,

    .unjamMaxWaitTimeMs = 500.0f,
    .moveDisplacementTick = 45 * DEGREE_TO_TICK,
    .unjamCycles = 4.0f,
    .unjamDisplacementTick = 45 * DEGREE_TO_TICK,
    .pauseAfterMoveTimeMs = 100.0f,
    .moveTimeMs = 125.0f,
    .setpointPosToleranceTick = 45 * DEGREE_TO_TICK,

    .jamCheckerToleranceTick = 5 * DEGREE_TO_TICK,
    .jamCheckerToleranceMs = 500,

    .isFeederInverted = true,

    .canBusMotors = tap::can::CanBus::CAN_BUS1,
};

template <target::RobotTarget R>
consteval FeederConfig getFeederConfig();

} // namespace control::feeder

#include "feeder_config_impl.hpp"

#endif