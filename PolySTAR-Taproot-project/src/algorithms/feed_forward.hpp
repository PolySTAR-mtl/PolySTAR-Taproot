#ifndef FEED_FORWARD_HPP_
#define FEED_FORWARD_HPP_

#include <cstdint>

namespace src
{
namespace algorithms
{
struct FeedForwardConfig
{
    float ks = 0.f; // Static friction compensation gain
    float kv = 0.f; // Velocity feedforward gain
    float kg = 0.f; // Gravity compensation gain
    float maxVelocity = 0.f;
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

    void setKs(float ks);
    void setKv(float kv);
    void setKg(float kg);
    void setMaxVel(float maxVel);

private:
    // gains and constants, to be set by the user
    FeedForwardConfig config_;
};

}  // namespace algorithms

}  // namespace src

#endif  // FEED_FORWARD_HPP_
