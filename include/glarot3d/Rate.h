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

#include <chrono>
#include <thread>

namespace glarot3d
{

/** Inspired by ros::Rate.
 */
class Rate
{
public:
  typedef std::chrono::duration<int,std::milli> milliseconds_type;

  /** Constructor.
   */ 
  Rate(double frequency) 
   : period_((int)(1000.0 / frequency)), start_(std::chrono::system_clock::now())
  {
  }

  /**
   */
  void sleep() 
  {
    // Computes the remaining time in the cycle
//    std::chrono::system_clock::time_point stop = start_ + period_; 
//    auto remaining = stop - chrono::system_clock::now();
//    auto remaining_ms = chrono::duration_cast<std::chrono::milliseconds>(diff);

    // Sleep
    std::this_thread::sleep_until(start_ + period_);
    start_ = std::chrono::system_clock::now();
  }

private:
  std::chrono::system_clock::time_point start_; 
  milliseconds_type period_;
};


}  // end of namespace glarot3d
