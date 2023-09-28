#ifndef GRAVITY_FEED_FORWARD_HPP_
#define GRAVITY_FEED_FORWARD_HPP_

#include <cstdint>
#include "feed_forward.hpp"

namespace src
{
namespace algorithms
{

class GravityFeedForward : public FeedForward
{
public:

    GravityFeedForward(const FeedForwardConfig &ffConfig, float x, float y);

    /**
     * Calculates the FeedForward input with gravity compensation.
     *
     * @param[in] velocity The desired velocity (in user-defined units) of the controlled system.
     */
    float calculate(float velocity, float angleDeg);

private:
    float offsetAngle;
};

}  // namespace algorithms

}  // namespace src

#endif  // GRAVITY_FEED_FORWARD_HPP_
