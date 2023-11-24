#pragma once
#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "chassis_constants.hpp"

class ChassisSubsystem : public tap::control::Subsystem 
{
public:
    ChassisSubsystem(tap::Drivers* drivers);

    ChassisSubsystem(const ChassisSubsystem&) = delete;
    ChassisSubsystem& operator=(const ChassisSubsystem&) = delete;

    ~ChassisSubsystem();
    
    void initialize() override;

    void refresh() override;

    void setLeftBackRpm(float rpm);
    void setLeftFrontRpm(float rpm);
    void setRightBackRpm(float rpm);
    void setRightFrontRpm(float rpm);

private:
    enum MOTOR_INDEX {
        MOTOR_LB = 0,
        MOTOR_LF = 1,
        MOTOR_RB = 2,
        MOTOR_RF = 3,
    };
    ///< Hardware constants, not specific to any particular feeder.
    static constexpr const char* const MOTOR_NAME[] = {"MOTOR LEFT BACK", "MOTOR LEFT FRONT", "MOTOR RIGHT BACK", "MOTOR RIGHT LEFT"};
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor* chassisMotors_[NUMBER_OF_MOTORS];

    modm::Pid<float>* pidMotors_[NUMBER_OF_MOTORS];

    float motorsRpm_[NUMBER_OF_MOTORS];
    // float feederDesiredPos;
    // uint32_t prevPidUpdate;

    // uint16_t feederOrigin;
    // bool feederCalibrated;

    // uint32_t prevDebugMessage = 0;

};