#ifndef AUTO_FEED_POLICY_HPP
#define AUTO_FEED_POLICY_HPP

#include "control/drivers/drivers.hpp"

namespace control::feeder
{
template <typename Subsystem>
class AutoFeedPolicy
{
public:
    AutoFeedPolicy(Subsystem* const feeder);

    ~AutoFeedPolicy();

    void initialize();

    void execute();

    void end(bool interrupt);

private:
    Subsystem* feeder_;
};

} // control::feeder

#include "auto_feed_policy_impl.hpp"

#endif //AUTO_FEED_POLICY_HPP