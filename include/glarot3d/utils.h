/**
 * GLAROT-3D - Geometric LAndmark relations ROTation-invariant 3D 
 * Copyright (C) 2017 Dario Lodi Rizzini.
 *
 * glarot3d is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * glarot3d is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with glarot3d.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <cinttypes>
#include <cmath>
#include <string>
#include <vector>


namespace glarot3d
{

// ----------------------------------------------
// I/O OPERATIONS
// ----------------------------------------------

/** Returns a list of files based on Unix-like GLOB. 
 */
void glob(const std::string globPath,std::vector<std::string>& matchingFiles);

/** Generates a filename dependent on date and time.
 */
std::string generateStampedString(const std::string prefix = "",const std::string postfix = "");


} // end of namespace
 
