#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "control/drivers/drivers.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"
#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/algorithms/cascaded_pid.hpp"

using turret::algorithms::CascadedPid;

namespace control
{
namespace turret
{

/**
 * Subsytem class for the turret. Controls both turret motors and sends data to
 * the computer vision system.
 */
class TurretSubsystem : public tap::control::Subsystem
{
public:

    /**
     * Constructs a new TurretSubsystem with default parameters specified in
     * the private section of this class.
     */
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
    const tap::motor::DjiMotor &getYawMotor() const { return *yawMotor; }
    const tap::motor::DjiMotor &getPitchMotor() const { return pitchMotor; }
    int64_t getYawNeutralPos() { return ACTIVE_TURRET_CONFIG.yawNeutralPos; }
    int64_t getPitchNeutralPos() { return ACTIVE_TURRET_CONFIG.pitchNeutralPos; }
    int64_t getYawUnwrapped() { return yawMotor->getEncoderUnwrapped(); }
    int64_t getPitchUnwrapped() { return pitchMotor.getEncoderUnwrapped(); }
    int getYawWrapped() { return yawMotor->getEncoderWrapped(); }
    int getPitchWrapped() { return pitchMotor.getEncoderWrapped(); }

    //setters
    void setIsSpin2WinMode(bool isSpin2WinMode) { m_isSpin2WinMode = isSpin2WinMode; }
    void setDesiredYawRpm(float desiredRpm) { desiredYawRpm = desiredRpm; }


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

    // Hardware interfaces
    src::Drivers *drivers;
    tap::motor::DjiMotor *yawMotor;
    tap::motor::DjiMotor pitchMotor;

    // Motor Controllers for position control
    CascadedPid cascadedPitchController;
    CascadedPid cascadedYawController;

    // Position setpoints for turret, in encoder ticks
    float yawDesiredPos;
    float pitchDesiredPos;

    // Time variables for fixed rate tasks
    uint32_t prevDebugUpdate;
    uint32_t prevControllerUpdate;
    uint32_t prevCVUpdate;

    // added functions and variables for yaw rpm during spin2win
    tap::algorithms::SmoothPid yawRpmPid;
    void updateRpmPid(tap::algorithms::SmoothPid* pid, tap::motor::DjiMotor* const motor, float desiredRpm, uint32_t dt);
    bool m_isSpin2WinMode = false;
    float desiredYawRpm = 0;

};  // class TurretSubsystem

}  // namespace turret

}  // namespace control

#endif  // TURRET_SUBSYSTEM_HPP_
