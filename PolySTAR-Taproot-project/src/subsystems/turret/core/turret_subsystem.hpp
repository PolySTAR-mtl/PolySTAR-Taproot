#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "control/drivers/drivers.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/algorithms/cascaded_pid.hpp"
#include "subsystems/turret/utils/modes/aim_mode.hpp"
#include "subsystems/turret/utils/modes/spin_mode.hpp"
#include "subsystems/sentry_general_constants.hpp"
#include "subsystems/turret/algorithms/imu_interpreter.hpp"



using turret::algorithms::CascadedPid;

namespace control::turret
{

/**
 * Subsytem class for the turret. Controls both turret motors and sends data to
 * the computer vision system.
 */
class TurretSubsystem : public tap::control::Subsystem
{
public:
    TurretSubsystem(src::Drivers *drivers, tap::motor::DjiMotor *yawMotor);

    TurretSubsystem(const TurretSubsystem &other) = delete;

    TurretSubsystem &operator=(const TurretSubsystem &other) = delete;

    ~TurretSubsystem() = default;

    void initialize() override;

    void refresh() override;

    // Position Setters
    void setAbsoluteOutput(uint16_t yaw, uint16_t pitch);
    void setAbsoluteOutputDegrees(float yaw, float pitch);
    void setRelativeOutput(float yawDelta, float pitchDelta);

    // Getters
    const tap::motor::DjiMotor& getYawMotor() const;
    const tap::motor::DjiMotor& getPitchMotor() const;
    int64_t getYawNeutralPos();
    int64_t getPitchNeutralPos();
    int64_t getYawUnwrapped();
    int64_t getPitchUnwrapped();
    int getYawWrapped();
    int getPitchWrapped();

    //setters
    void setIsSpin2WinMode(bool isSpin2WinMode);
    void setDesiredYawRpm(float desiredRpm);

    /**
     * Aim policy methods. Definition can be found in the implementation file.
     */
    template <AimMode A, SpinMode S>
    void initializeAiming();

    template <AimMode A, SpinMode S>
    void executeAiming();

    template <AimMode A, SpinMode S>
    void stopAiming();

private:
    // Controller Functions
    void runYawController(uint32_t dt);
    void runPitchController(uint32_t dt);

    // Debugging and Communication
    void sendCVUpdate();
    void sendDebugInfo(bool sendYaw, bool sendPitch);

    // Methods used when tuning the inner loop of the cascaded PID controller
    void yawInnerLoopTest(uint32_t dt, float velSetpoint, float threshold);
    void pitchInnerLoopTest(uint32_t dt, float velSetpoint, float threshold);
    void sendTuningDebugInfo(bool sendYaw, bool sendPitch, float velSetpoint, float threshold);

    // Method for yaw rpm during spin2win
    void updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm, uint32_t dt);
    
    // Hardware interfaces
    src::Drivers* drivers_;
    tap::motor::DjiMotor* yawMotor_;
    tap::motor::DjiMotor pitchMotor_;

    // Motor Controllers for position control
    CascadedPid cascadedPitchController_;
    CascadedPid cascadedYawController_;

    // Position setpoints for turret, in encoder ticks
    float yawDesiredPos_;
    float pitchDesiredPos_;

    // Time variables for fixed rate tasks
    uint32_t prevDebugUpdate_;
    uint32_t prevControllerUpdate_;
    uint32_t prevCVUpdate_;

    // Variables for yaw rpm during spin2win
    tap::algorithms::SmoothPid yawRpmPid_;
    bool isSpin2WinMode_;
    float desiredYawRpm_;

    tap::arch::MilliTimeout startMatchTimeout_;
    algorithms::ImuInterpreter imuInterpreter_;

};  // class TurretSubsystem

}  // namespace control::turret

#include "turret_subsystem_impl.hpp"

#endif  // TURRET_SUBSYSTEM_HPP_
