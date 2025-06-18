#include "tap/motor/dji_motor.hpp"
#include "control/drivers/drivers_singleton.hpp"

using src::DoNotUse_getDrivers;

static src::driversFunc drivers = src::DoNotUse_getDrivers;

// Turret motors
tap::motor::DjiMotor yawMotor(drivers(), tap::motor::MOTOR6, tap::can::CanBus::CAN_BUS1, true, "yaw motor");

// TODO: Initialize chassis motors

// TODO: Initialize feeder motors

// TODO: Initialize flywheel motors

// TODO: Create structs to pass to subsystems
