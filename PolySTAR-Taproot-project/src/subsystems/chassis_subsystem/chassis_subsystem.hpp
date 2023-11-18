#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "constants/chassis_constants.hpp"

namespace control {
    namespace chassis {
        class ChassisSubsystem : public tap::control::Subsystem {
        public:
            ChassisSubsystem(tap::Drivers *drivers)
                : tap::control::Subsystem(drivers),
                    chassisMotorFrontLeft(drivers, CHASSIS_MOTOR_FRONT_LEFT, CAN_BUS_CHASSIS_MOTORS, IS_CHASSIS_LEFT_MOTORS_INVERTED, "chassis motor front left"),
                    chassisMotorFrontRight(drivers, CHASSIS_MOTOR_FRONT_RIGHT, CAN_BUS_CHASSIS_MOTORS, IS_CHASSIS_RIGHT_MOTORS_INVERTED, "chassis motor front right"),
                    chassisMotorBackLeft(drivers, CHASSIS_MOTOR_BACK_LEFT, CAN_BUS_CHASSIS_MOTORS, IS_CHASSIS_LEFT_MOTORS_INVERTED, "chassis motor back left"),
                    chassisMotorBackRight(drivers, CHASSIS_MOTOR_BACK_RIGHT, CAN_BUS_CHASSIS_MOTORS, IS_CHASSIS_RIGHT_MOTORS_INVERTED, "chassis motor back right"),
                    chassisPidFrontLeft(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
                    chassisPidFrontRight(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
                    chassisPidBackLeft(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
                    chassisPidBackRight(CHASSIS_PID_KP,CHASSIS_PID_KI,CHASSIS_PID_KD,CHASSIS_PID_MAX_ERROR_SUM,CHASSIS_PID_MAX_OUTPUT),
                    DesiredRpmFrontLeft(0.0f),
                    DesiredRpmFrontRight(0.0f),
                    DesiredRpmBackLeft(0.0f),
                    DesiredRpmBackRight(0.0f)
            {}

            ~ChassisSubsystem() = default;

            /**
             * Called once when the subsystem is added to the scheduler.
             */
            void initialize() override;

            /**
             * Will be called periodically whenever the CommandScheduler runs.
             */
            void refresh() override;

            /** Public function used to set the desired RPM of the chassis motor. Used by commands. */
            void setDesiredOutputFrontLeft(float rpmFrontLeft);

            /** Public function used to set the desired RPM of the chassis motor. Used by commands. */
            void setDesiredOutputFrontRight(float rpmFrontRight);

            /** Public function used to set the desired RPM of the chassis motor. Used by commands. */
            void setDesiredOutputBackLeft(float rpmBackLeft);

            /** Public function used to set the desired RPM of the chassis motor. Used by commands. */
            void setDesiredOutputBackRight(float rpmBackRight);

        private:
        // Components (e.g. motor controllers and sensors) should generally be
        // declared private and exposed only through public methods.

        // Motor IDs.  Use these to interact with any dji style motors.
        static constexpr tap::motor::MotorId CHASSIS_MOTOR_FRONT_LEFT = tap::motor::MOTOR1;  
        static constexpr tap::motor::MotorId CHASSIS_MOTOR_FRONT_RIGHT = tap::motor::MOTOR2;
        static constexpr tap::motor::MotorId CHASSIS_MOTOR_BACK_LEFT = tap::motor::MOTOR3;   
        static constexpr tap::motor::MotorId CHASSIS_MOTOR_BACK_RIGHT = tap::motor::MOTOR4;  
        
        // CAN bus to which the motor is connected
        static constexpr tap::can::CanBus CAN_BUS_CHASSIS_MOTORS = tap::can::CanBus::CAN_BUS1;

        ///< Motors.  Use these to interact with any dji style motors.
        tap::motor::DjiMotor chassisMotorFrontLeft;
        tap::motor::DjiMotor chassisMotorFrontRight;
        tap::motor::DjiMotor chassisMotorBackLeft;
        tap::motor::DjiMotor chassisMotorBackRight;

        ///< PID controller for rpm feedback from motor
        modm::Pid<float> chassisPidFrontLeft;
        modm::Pid<float> chassisPidFrontRight;
        modm::Pid<float> chassisPidBackLeft;
        modm::Pid<float> chassisPidBackRight;

        ///< Desired rpm for the motor. Acts as a setpoint for the subsystem.
        float DesiredRpmFrontLeft;
        float DesiredRpmFrontRight;
        float DesiredRpmBackLeft;
        float DesiredRpmBackRight;

        };
    }
}