#ifndef NORMAL_FEED_POLICY_HPP
#define NORMAL_FEED_POLICY_HPP

namespace control::feeder
{
template <typename Subsystem>
class NormalFeedPolicy
{
public:
    NormalFeedPolicy(Subsystem* const feeder);

    ~NormalFeedPolicy();

    void initialize();

    void execute();

    void end(bool interrupt);
private:
    Subsystem* const feeder_;
};

} // control::feeder

#include "normal_feed_policy_impl.hpp"

#endif // NORMAL_FEED_POLICY_HPP
