#include "subsystems/turret/algorithms/imu_interpreter.hpp"

#include <numbers>
#include <cmath>

#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

#include "conversions/rotation_conversion.hpp"

namespace algorithms
{
    ImuInterpreter::ImuInterpreter(src::Drivers *drivers)
        : drivers{drivers}
        , turretYawRPM{0.f}
        , samplingWindowStartTime{0}
        , lastImuDataTime{0}
        , hasSamplingWindow{false}
        , chassisRotationSpeed{0.f}
        , gzSamplingCount{0}
        , gzSamplingSum{0.f}
    {}

    void ImuInterpreter::update(float xInput) {
        static constexpr std::uint32_t GZ_SAMPLING_WINDOW_MS{20};
        static constexpr std::uint32_t IMU_DATA_TIMEOUT_US{50'000};
        static constexpr float GZ_THRESHOLD_RADIANS_PER_SECOND{0.5f * std::numbers::pi_v<float> / 180.f};

        const std::uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
        const std::uint32_t currentImuDataTime = drivers->mpu6500.getPrevIMUDataReceivedTime();

        if (drivers->mpu6500.getImuState() !=
            tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_CALIBRATED ||
            tap::arch::clock::getTimeMicroseconds() - currentImuDataTime > IMU_DATA_TIMEOUT_US) {
            samplingWindowStartTime = currentUpdate;
            lastImuDataTime = currentImuDataTime;
            hasSamplingWindow = false;
            chassisRotationSpeed = 0.f;
            gzSamplingCount = 0;
            gzSamplingSum = 0.f;
            turretYawRPM = 0.f;
            return;
        }

        if (!hasSamplingWindow) {
            samplingWindowStartTime = currentUpdate;
            hasSamplingWindow = true;
        }

        if (currentImuDataTime != lastImuDataTime) {
            gzSamplingSum += drivers->mpu6500.getGz();
            ++gzSamplingCount;
            lastImuDataTime = currentImuDataTime;
        }

        if (currentUpdate - samplingWindowStartTime >= GZ_SAMPLING_WINDOW_MS && gzSamplingCount > 0) {
            const float gzAverage = gzSamplingSum / gzSamplingCount;

            if (std::abs(gzAverage) > GZ_THRESHOLD_RADIANS_PER_SECOND) {
                chassisRotationSpeed = gzAverage;
            }
            else {
                chassisRotationSpeed = 0;
            }

            samplingWindowStartTime = currentUpdate;
            gzSamplingSum = 0.f;
            gzSamplingCount = 0;
        }

        const float stabilizedXInput = std::abs(xInput) >= control::turret::TURRET_DEAD_ZONE ? xInput : 0.f;
        const float manualYawFactor = control::turret::X_INPUT_STABILIZATION_CONSTANT * stabilizedXInput;

        const float desiredYawRadPerSecond = (control::turret::ACTIVE_TURRET_CONFIG.gzStabilizationFactor - manualYawFactor) * chassisRotationSpeed;

        turretYawRPM = conversions::radiansPerSecondToRpm(desiredYawRadPerSecond);
    }

    float ImuInterpreter::getTurretYawRPM() const {
        return turretYawRPM;
    }
} // namespace algorithms