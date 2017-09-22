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
#include <glarot3d/utils.h>
#include <glob.h>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace glarot3d
{

// ----------------------------------------------
// I/O OPERATIONS
// ----------------------------------------------

void glob(const std::string globPath,std::vector<std::string>& matchingFiles)
{
  glob_t glob_result;
  matchingFiles.clear();
  ::glob(globPath.c_str(),GLOB_TILDE,NULL,&glob_result);
  for(unsigned int i=0; i < glob_result.gl_pathc; ++i) {
    matchingFiles.push_back(std::string(glob_result.gl_pathv[i]));
  }
  globfree(&glob_result);
}

std::string generateStampedString(const std::string prefix,const std::string postfix)
{
  boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time(); 
  std::ostringstream formatter;
  std::string formatstring = prefix + "%Y%m%d_%H%M_%S" + postfix;
  formatter.imbue(std::locale(std::cout.getloc(), new boost::posix_time::time_facet(formatstring.c_str())));
  formatter << now;
  return formatter.str();
}

}  // end of namespace
