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

#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

#include "drivers/drivers.hpp"

namespace control {
void initSubsystemCommands(src::Drivers* drivers);
}

#endif
