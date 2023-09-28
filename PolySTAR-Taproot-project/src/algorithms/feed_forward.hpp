#ifndef FEED_FORWARD_HPP_
#define FEED_FORWARD_HPP_

#include <cstdint>

namespace src
{
namespace algorithms
{
struct FeedForwardConfig
{
    float ks = 0.0f; // Static friction compensation gain
    float kv = 0.0f; // Velocity feedforward gain
    float kg = 0.0f; // Gravity compensation gain
    float maxVelocity = 0.0f;
};

class FeedForward
{
public:
    FeedForward(const FeedForwardConfig &ffConfig);

    /**
     * Calculates the FeedForward input.
     *
     * @param[in] velocity The desired velocity (in user-defined units) of the controlled system.
     */
    float calculate(float velocity);

    inline void setKs(float ks) { config.ks = ks; }
    inline void setKv(float kv) { config.kv = kv; }
    inline void setKg(float kg) { config.kg = kg; }
    inline void setMaxVel(float maxVel) { config.maxVelocity = maxVel; }

protected:
    // gains and constants, to be set by the user
    FeedForwardConfig config;

    static constexpr float DEG_TO_RAD = 0.0174533;

};

}  // namespace algorithms

}  // namespace src

#endif  // FEED_FORWARD_HPP_
