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
#include <glarot3d/Glarot3d.h>
#include <cmath>

namespace glarot3d
{

  /**********************************************
   * CONSTRUCTOR AND INFORMATION
   **********************************************/

Glarot3d::Glarot3d(int angleNum,int rangeNum,double rangeRes) 
: acc_(6*angleNum*angleNum*rangeNum, 0), angleNum_(angleNum), faceNum_(6*angleNum_*angleNum_),
    rangeNum_(rangeNum), rangeRes_(rangeRes)
 {
 }

Glarot3d::~Glarot3d() { }

void Glarot3d::reset(int angleNum,int rangeNum,double rangeRes) 
{
  acc_.resize(6*angleNum*angleNum*rangeNum);
  std::fill(acc_.begin(),acc_.end(),0);
  angleNum_ = angleNum; 
  faceNum_ = 6*angleNum_*angleNum_;
  rangeNum_ = rangeNum;
  rangeRes_ = rangeRes;
}

void Glarot3d::clear()
{
  for (auto& a : acc_) {
    a = 0;
  }
}

  /**********************************************
   * COMPARE FUNCTIONS
   **********************************************/

Glarot3d::Accumulator Glarot3d::normL1(const Glarot3d& signature) const
{
  if (angleNum_ != signature.angleNum() || rangeNum_ != signature.rangeNum_) {
    std::cerr << __FILE__ << "," << __LINE__ << ": non-matching size (angleNum " << angleNum_ << ", rangeNum " << rangeNum_ << ") "
      << "compared with signature (angleNum " << signature.angleNum() << ", rangeNum " << signature.rangeNum() << ")" << std::endl;
    return 100000;
  }
  Accumulator tot = 0;
  for (Index idx = 0; idx < acc_.size(); ++idx) {
    tot += abs(value(idx) - signature.value(idx));
  }
  return tot;
}

Glarot3d::Accumulator Glarot3d::normL1(const Glarot3d& signature,const CubeSymmetry& sym) const
{
  if (angleNum_ != signature.angleNum() || rangeNum_ != signature.rangeNum_) {
    std::cerr << __FILE__ << "," << __LINE__ << ": non-matching size (angleNum " << angleNum_ << ", rangeNum " << rangeNum_ << ") "
      << "compared with signature (angleNum " << signature.angleNum() << ", rangeNum " << signature.rangeNum() << ")" << std::endl;
    return 100000;
  }

  Accumulator tot = 0;
  for (Index f = 0; f < 6; ++f) {
    for (Index u = 0; u < angleNum_; ++u) {
      for (Index v = 0; v < angleNum_; ++v) {
        for (Index r = 0; r < rangeNum_; ++r) {
          tot += abs( value(f,u,v,r) - signature.value(f,u,v,r,sym) );
        }
      }
    }
  }
  return tot;
}

Glarot3d::Accumulator Glarot3d::normL1Min(const Glarot3d& signature,CubeSymmetry& symMin) const
{
  if (angleNum_ != signature.angleNum() || rangeNum_ != signature.rangeNum()) {
    std::cerr << __FILE__ << "," << __LINE__ << ": non-matching size (angleNum " << angleNum_ << ", rangeNum " << rangeNum_ << ") "
      << "compared with signature (angleNum " << signature.angleNum() << ", rangeNum " << signature.rangeNum() << ")" << std::endl;
    return 100000;
  }

  Accumulator normMin = 1000000;
  Accumulator normVal;
  for (Index i = 0; i < CubeSymmetry::SYMMETRY_NUM; ++i) {
    CubeSymmetry sym = CubeSymmetry::getSymmetry(i);
    normVal = normL1(signature,sym);
    if (normVal < normMin) {
      normMin = normVal;
      symMin = sym;
    }
  }
  return normMin;
}

  /**********************************************
   * SUPPORT FUNCTIONS
   **********************************************/

Glarot3d::Index Glarot3d::convertXYZToIdx(double x,double y,double z,Index& f,Index& u,Index& v,Index& r) const
{
  double absX = fabs(x);
  double absY = fabs(y);
  double absZ = fabs(z);
  bool isXPositive = (x > 0);
  bool isYPositive = (y > 0);
  bool isZPositive = (z > 0);
  double uc, vc, rho, axisMax;
  Index idx;

  // POSITIVE X
  if (isXPositive && absX >= absY && absX >= absZ) {
    axisMax = absX;
    f = CubeSymmetry::XP;
    uc = selectCoordinate(CubeSymmetry::axisU(f),x,y,z);
    vc = selectCoordinate(CubeSymmetry::axisV(f),x,y,z);
  }
  // NEGATIVE X
  if (!isXPositive && absX >= absY && absX >= absZ) {
    axisMax = absX;
    f = CubeSymmetry::XN;
    uc = selectCoordinate(CubeSymmetry::axisU(f),x,y,z);
    vc = selectCoordinate(CubeSymmetry::axisV(f),x,y,z);
  }
  // POSITIVE Y
  if (isYPositive && absY >= absX && absY >= absZ) {
    axisMax = absY;
    f = CubeSymmetry::YP;
    uc = selectCoordinate(CubeSymmetry::axisU(f),x,y,z);
    vc = selectCoordinate(CubeSymmetry::axisV(f),x,y,z);
  }
  // NEGATIVE Y
  if (!isYPositive && absY >= absX && absY >= absZ) {
    // u (0 to 1) goes from -x to +x
    // v (0 to 1) goes from -z to +z
    axisMax = absY;
    f = CubeSymmetry::YN;
    uc = selectCoordinate(CubeSymmetry::axisU(f),x,y,z);
    vc = selectCoordinate(CubeSymmetry::axisV(f),x,y,z);
  }
  // POSITIVE Z
  if (isZPositive && absZ >= absX && absZ >= absY) {
    // u (0 to 1) goes from -x to +x
    // v (0 to 1) goes from -y to +y
    axisMax = absZ;
    f = CubeSymmetry::ZP;
    uc = selectCoordinate(CubeSymmetry::axisU(f),x,y,z);
    vc = selectCoordinate(CubeSymmetry::axisV(f),x,y,z);
  }
  // NEGATIVE Z
  if (!isZPositive && absZ >= absX && absZ >= absY) {
    // u (0 to 1) goes from +x to -x
    // v (0 to 1) goes from -y to +y
    axisMax = absZ;
    f = CubeSymmetry::ZN;
    uc = selectCoordinate(CubeSymmetry::axisU(f),x,y,z);
    vc = selectCoordinate(CubeSymmetry::axisV(f),x,y,z);
  }

  // Computes the face grid indices u and v
  double ua = (2.0 * atan(uc / axisMax) / M_PI +  0.5);
  double va = (2.0 * atan(vc / axisMax) / M_PI +  0.5);
  u = (Index)floor(angleNum_ * (2.0 * atan(uc / axisMax) / M_PI +  0.5));
  v = (Index)floor(angleNum_ * (2.0 * atan(vc / axisMax) / M_PI +  0.5));
  if (u == angleNum_) u--;
  if (v == angleNum_) v--;
  if (v < 0.0 || v >= angleNum_ || u < 0.0 || u >= angleNum_) {
    std::cerr << __FILE__ << "," << __LINE__ << ": v " << v << " or u " << u << " out of bound " << angleNum_ << std::endl;
  }
  assert(u < angleNum_);
  assert(v < angleNum_);

  // Computes range index r
//  std::cout << "rho " << rho << ": x " << x << " y " << y << " z " << z << std::endl; 
  rho = sqrt(x*x + y*y + z*z);
  r = (Index)floor(rho / rangeRes_);
  if (r >= rangeNum_) {
//    std::cerr << __FILE__ << "," << __LINE__ << ": out of range vector length: "
//      << "r = floor(" << rho << "/" << rangeRes_ << ") = " << r << " >= " << rangeNum_ << std::endl;
    return acc_.size();
  }

  // Compute total index
  idx = (f * angleNum_ * angleNum_ + u * angleNum_ + v) * rangeNum_ + r;
  return idx;
}

void Glarot3d::rotateUV(Index uin,Index vin,Index rot,Index& uout,Index& vout) const
{
  Index rotm = (rot % 4);
  if (rotm == 0) {
    uout = uin;
    vout = vin;
  }
  else if (rotm == 1) {
    uout = angleNum_ - 1 - vin;
    vout = uin;
  }
  else if (rotm == 1) {
    uout = vin;
    vout = angleNum_ - 1 - uin;
  }
  else if (rotm == 2) {
    uout = angleNum_ - 1 - uin;
    vout = angleNum_ - 1 - vin;
  }
  else if (rotm == 3) {
    uout = vin;
    vout = angleNum_ - 1 - uin;
  }
  else {
    assert(0);
  }
}

void Glarot3d::print(std::ostream& out,CubeSymmetry sym) const
{
  Glarot3d::Index f, u, v, r;
  Accumulator tot;

//  for (f = 0; f < 6; ++f) {
//    out << "face " << CubeSymmetry::faceName(f) << " in " << CubeSymmetry::faceName(sym.face(f)) << ":\n";
//    for (u = angleNum_-1; 0 <= u && u < angleNum_; --u) {
//      for (v = 0; v < angleNum_; ++v) {
//        checkIdx(__FILE__,__LINE__,f,u,v,0);
//        out << "\t" << value(f,u,v,0,sym);
//      }
//      out << "\n";
//    }
//  }
  for (f = 0; f < 6; ++f) {
    out << "face " << CubeSymmetry::faceName(f) << " in " << CubeSymmetry::faceName(sym.face(f)) << " (rot " << sym.faceRotation(f) << "):\n";
    for (v = angleNum_-1; 0 <= v && v < angleNum_; --v) {
      for (u = 0; u < angleNum_; ++u) {
        tot = 0;
        for (r = 0; r < rangeNum_; ++r) {
          checkIdx(__FILE__,__LINE__,f,u,v,r);
          tot += value(f,u,v,r,sym);
        }
        out << "\t" << tot;
      }
      out << "\n";
    }
    out << "^ " << sym.faceName(sym.axisV(sym.face(f))) << "\t" << sym.faceName(sym.axisU(sym.face(f))) << " >" << std::endl;
  }
}

void Glarot3d::save(std::ostream& out) const
{
  out << angleNum_ << " " << rangeNum_ << " " << rangeRes_ << " ";
}

double Glarot3d::selectCoordinate(FaceIdx f,double x,double y,double z)
{
  switch(f) {
    case CubeSymmetry::XP: return  x;
    case CubeSymmetry::XN: return -x;
    case CubeSymmetry::YP: return  y;
    case CubeSymmetry::YN: return -y;
    case CubeSymmetry::ZP: return  z;
    case CubeSymmetry::ZN: return -z;
  }
  return 0.0;
}

} // end of namespace

