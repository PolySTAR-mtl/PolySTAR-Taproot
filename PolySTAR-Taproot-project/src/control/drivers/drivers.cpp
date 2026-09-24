#include "drivers.hpp"

namespace src
{

Drivers::Drivers()
    : tap::Drivers{}
    , controlInterface{this}
    , cvHandler{this}
{}

} // namespace src