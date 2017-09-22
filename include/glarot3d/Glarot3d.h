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

#include <vector>
#include <cstdint>
#include <cassert>
#include <Eigen/Dense>
#include <glarot3d/CubeSymmetry.h>


namespace glarot3d
{

/**
 * Class Glarot3d implements the 3D geometric signature describing the spatial distribution
 * of a point set. 
 */
class Glarot3d
{
public:
  typedef int Accumulator;
  typedef CubeSymmetry::FaceIdx FaceIdx;
  typedef CubeSymmetry::Index Index;

  /**********************************************
   * CONSTRUCTOR AND INFORMATION
   **********************************************/

  /** Constructor with the desired grid dimensions.
   */
  Glarot3d(int angleNum,int rangeNum,double rangeRes);

  /** Destructor.
   */
  ~Glarot3d();

  /**
   */
  void reset(int angleNum,int rangeNum,double rangeRes);

  /** Clears the content of histogram.
   */
  void clear();

  /** Returns the face grid size. 
   */
  Index angleNum() const { return angleNum_; }

  /** Returns the number of range bins. 
   */
  Index rangeNum() const { return rangeNum_; }

  /**********************************************
   * ACCESS TO VALUES
   **********************************************/

  /** Returns true if point (x,y,z) lies in the histogram domain.
   */
  bool inside(double x,double y,double z) 
  {
    Index idx, f, u, v, r;
    idx = convertXYZToIdx(x,y,z,f,u,v,r);
    if (f >= 6 || u >= angleNum_ || v >= angleNum_ || r >= rangeNum_) {
      return false;
    }
    return true;
  }

  bool inside(const Eigen::Vector3d& p) 
  {
    return inside(p.x(),p.y(),p.z());
  }

  /** Returns a reference to the bin corresponding to (x,y,z). 
   */
  Accumulator& value(double x,double y,double z) 
  {
    Index idx, f, u, v, r;
    idx = convertXYZToIdx(x,y,z,f,u,v,r);
    assert(idx < acc_.size());
    return acc_[idx];
  }

  /** Returns the value reference to the bin corresponding to (x,y,z). 
   */
  Accumulator value(double x,double y,double z) const
  {
    Index idx, f, u, v, r;
    idx = convertXYZToIdx(x,y,z,f,u,v,r);
    assert(idx < acc_.size());
    return acc_[idx];
  }

  /** Returns a reference to the bin corresponding to p. 
   */
  Accumulator& value(const Eigen::Vector3d& p) 
  {
    return value(p.x(),p.y(),p.z());
  }

  /** Returns a const reference to the bin corresponding to p. 
   */
  Accumulator value(const Eigen::Vector3d& p) const
  {
    return value(p.x(),p.y(),p.z());
  }

  /** Returns the cell corresponding to the given indices: 
   * face f, axis u, axis v, range r.
   */
  Accumulator value(Index f,Index u,Index v,Index r) const
  {
    checkIdx(__FILE__,__LINE__,f,u,v,r);
    Index idx = (f * angleNum_ * angleNum_ + u * angleNum_ + v) * rangeNum_ + r;
    assert(idx < acc_.size());
    return acc_[idx];
  }

  /** Returns the cell corresponding to the given indices: 
   * face f, axis u, axis v, range r.
   */
  Accumulator& value(Index f,Index u,Index v,Index r) 
  {
    checkIdx(__FILE__,__LINE__,f,u,v,r);
    Index idx = (f * angleNum_ * angleNum_ + u * angleNum_ + v) * rangeNum_ + r;
    assert(idx < acc_.size());
    return acc_[idx];
  }

  /** Returns the cell corresponding to the unique index.
   */
  Accumulator value(Index idx) const
  {
    if (idx >= acc_.size()) {
      std::cerr << __FILE__ << "," << __LINE__ << ": out-of-range face index" << std::endl;
    }
    return acc_[idx];
  }

  /** Returns the cell corresponding to the given indices: 
   * face f, axis u, axis v, range r.
   */
  Accumulator value(Index f,Index u,Index v,Index r,const CubeSymmetry& sym) const
  {
    checkIdx(__FILE__,__LINE__,f,u,v,r);

    Index f2, u2, v2, rot;
    f2 = (Index)sym.face(f);
    rot = sym.faceRotation(f);
    rotateUV(u,v,rot,u2,v2);

    Index idx = (f2 * angleNum_ * angleNum_ + u2 * angleNum_ + v2) * rangeNum_ + r;
    assert(idx < acc_.size());
    return acc_[idx];
  }

  /** Returns the cell corresponding to the given indices: 
   * face f, axis u, axis v, range r.
   */
  Accumulator& value(Index f,Index u,Index v,Index r,const CubeSymmetry& sym)
  {
    checkIdx(__FILE__,__LINE__,f,u,v,r);

    Index f2, u2, v2, rot;
    f2 = (Index)sym.face(f);
    rot = sym.faceRotation(f);
    rotateUV(u,v,rot,u2,v2);

    Index idx = (f2 * angleNum_ * angleNum_ + u2 * angleNum_ + v2) * rangeNum_ + r;
    assert(idx < acc_.size());
    return acc_[idx];
  }

  /**********************************************
   * COMPARE FUNCTIONS
   **********************************************/

  /** Returns the L1 norm to another Glarot3d feature (with same dimensions!). 
   * This comparison does not involve rotation.
   */
  Accumulator normL1(const Glarot3d& signature) const;

  /** Returns the L1 norm to another rotated Glarot3d feature (with same dimensions!). 
   */
  Accumulator normL1(const Glarot3d& signature,const CubeSymmetry& sym) const;

  /** Returns the minimum L1 norm to another rotated Glarot3d feature by trying all rotations. 
   */
  Accumulator normL1Min(const Glarot3d& signature,CubeSymmetry& symMin) const;

  /**********************************************
   * SUPPORT FUNCTIONS
   **********************************************/

  /** Converts 3D points into an accumulator index direction.
   */
  Index convertXYZToIdx(double x,double y,double z,Index& f,Index& u,Index& v,Index& r) const;

  /** Returns the norm for the rotated face index. 
   */ 
//  Index rotateUV(Index uin,Index vin,Index rot,Index& uout,Index& vout) const;

  /** Returns the coordinate associated to the given face. 
   * Example: (x, y, z) with face XP it returns x, with face XN it returns -x, etc.
   */
  static double selectCoordinate(FaceIdx f,double x,double y,double z);

  /** Returns the swapped indices (u,v) according to the face rotation.
   * Discrete rotation is set to 0, 1, 2 or 3 since it represents rotation of rot*90 deg. 
   */
  void rotateUV(Index uin,Index vin,Index rot,Index& uout,Index& vout) const;

  bool checkIdx(const char* file,int line,Index f,Index u,Index v,Index r) const
  {
    bool inRange = true;
    if (f >= 6) {
      std::cerr << file << "," << line << ": out-of-range face" << " >= 6" << std::endl;
      inRange = false;
    }
    if (u >= angleNum_) {
      std::cerr << file << "," << line << ": out-of-range u " << u << " >= " << angleNum_ << std::endl;
      inRange = false;
    }
    if (v >= angleNum_) {
      std::cerr << file << "," << line << ": out-of-range v " << v << " >= " << angleNum_ << std::endl;
      inRange = false;
    }
    if (r >= rangeNum_) {
      std::cerr << file << "," << line << ": out-of-range r " << r << " >= " << rangeNum_ << std::endl;
      inRange = false;
    }
    return inRange;
  }

  /**********************************************
   * I/O
   **********************************************/

  /** Print face grids content to output.
   */
  void print(std::ostream& out) const
  {
    CubeSymmetry sym(CubeSymmetry::IDENTITY_CODE);
    print(out,sym);
  }

  /** Print face grids content to output subject to CubeSymmetry sym.
   */
  void print(std::ostream& out,CubeSymmetry sym) const;

  /** Saves to stream. 
   */
  void save(std::ostream& out) const;

  /** Loads from stream. 
   */
  void load(std::ostream& in);

private:
  std::vector<Accumulator> acc_;
  int angleNum_;
  int faceNum_;
  int rangeNum_;
  double rangeRes_;
};

} // end of namespace glarot3d


