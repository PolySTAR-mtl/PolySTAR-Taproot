#include "subsystems/turret/algorithms/imu_interpreter.hpp"

#include <numbers>
#include <cmath>

#include "subsystems/turret/config/turret_config.hpp"
#include "subsystems/turret/config/constants/turret_constants.hpp"

namespace algorithms
{
    ImuInterpreter::ImuInterpreter(src::Drivers *drivers)
        : drivers{drivers}
        , turretYawRPM{0.f}
        , lastUpdateTime{0}
        , chassisRotationSpeed{0.f}
        , gzSamplingCount{0}
        , gzSamplingSum{0.f}
    {}

    void ImuInterpreter::update(const float xInput) {
        static constexpr std::uint32_t GZ_SAMPLING_WINDOW_MS = 20;

        const float gZ = drivers->mpu6500.getGz();
        gzSamplingSum += gZ;
        ++gzSamplingCount;

        const std::uint32_t currentUpdate = tap::arch::clock::getTimeMilliseconds();
        const std::uint32_t timeDelta = currentUpdate - lastUpdateTime;
        
        if (timeDelta >= GZ_SAMPLING_WINDOW_MS) {
            static constexpr float GZ_THRESHOLD_RADIANS_PER_SECOND = 0.5f * std::numbers::pi_v<float> / 180.0f; // Threshold for considering the Gz value significant
            
            lastUpdateTime = currentUpdate;
            
            const float gzAverage = gzSamplingSum / gzSamplingCount;

            if (std::abs(gzAverage) > GZ_THRESHOLD_RADIANS_PER_SECOND) {
                chassisRotationSpeed = gzAverage;
            }
            else {
                chassisRotationSpeed = 0;
            }

            gzSamplingSum = 0.f;
            gzSamplingCount = 0;
        }

        turretYawRPM = (control::turret::ACTIVE_TURRET_CONFIG.gzStabilizationFactor - control::turret::X_INPUT_STABILIZATION_CONSTANT * xInput) * chassisRotationSpeed;
    }

    float ImuInterpreter::getTurretYawRPM() const {
        return turretYawRPM;
    }
} // namespace algorithms