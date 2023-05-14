/*
 * Copyright (c) 2022-2023 PolySTAR-Mtl
 *
 * This file is part of PolySTAR-Taproot.
 *
 * PolySTAR-Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PolySTAR-Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PolySTAR-Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "tap/drivers.hpp"
#include "control/control_interface.hpp"
#include "communication/cv_handler.hpp"

namespace src
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : 
        tap::Drivers(),
        controlInterface(this),
        cvHandler(this)
    {
    }

public:
    control::ControlInterface controlInterface;
    communication::cv::CVHandler cvHandler;
};  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
