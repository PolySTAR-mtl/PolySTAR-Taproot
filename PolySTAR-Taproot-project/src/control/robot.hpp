#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "config/control_config.hpp"
#include "robot_target.hpp"

namespace control {

template <target::RobotTarget T>
class Robot
{
public:
    explicit Robot(src::Drivers *drivers);

    void initialize();

private:
    ControlConfig<T> config_;
};

extern template class Robot<target::RobotTarget::Standard>;

}


#endif