/**
 * GLAROT-3D - Geometric LAndmark relations ROTation-invariant 3D 
 * Copyright (C) 2017 Dario Lodi Rizzini on code from Fabjan Kallasi.
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
#ifndef TIMER_H
#define	TIMER_H 

#include <chrono>

typedef std::chrono::high_resolution_clock HRClock;

namespace glarot3d
{

struct Intervall {
  /// Current time in [unit]
  int nano;
  int micro;
  int milli;
  int sec;
  int min;
  int hour;
  
  Intervall(const HRClock::duration& dur) {
    nano = std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count();
    micro = std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
    milli = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
    min = std::chrono::duration_cast<std::chrono::minutes>(dur).count();
    hour = std::chrono::duration_cast<std::chrono::hours>(dur).count();
  }

  Intervall(int _nano = 0, int _micro = 0, int _milli = 0, int _sec = 0, int _min = 0, int _hour = 0) {
    hour = _hour;
    micro = _micro;
    milli = _milli;
    min = _min;
    nano = _nano;
    sec = _sec;
  }

  friend Intervall operator-(const Intervall& lv, const Intervall& rv) {
    return Intervall( lv.nano - rv.hour,
                      lv.micro - rv.micro,
                      lv.milli - rv.milli,
                      lv.sec - rv.sec,
                      lv.min - rv.min,
                      lv.hour - rv.hour
                    );
  }
  friend Intervall operator+(const Intervall& lv, const Intervall& rv) {
    return Intervall( lv.nano + rv.hour,
                      lv.micro + rv.micro,
                      lv.milli + rv.milli,
                      lv.sec + rv.sec,
                      lv.min + rv.min,
                      lv.hour + rv.hour
                    );
  }

//  Intervall operator +(const Intervall& lv, const Intervall& rv) {
//    return result;
//  }
private:
};

class Timer {
public:

  Timer() {
    start();
  }

  void start() {
    m_timePoint = HRClock::now(); 
  }

  Intervall getTime() {
    HRClock::duration dur = HRClock::now() - m_timePoint;
    return Intervall(dur);
  }

private:
  HRClock::time_point m_timePoint;
};

} // end of namespace glarot3d

#endif	/* TIMER_H */

