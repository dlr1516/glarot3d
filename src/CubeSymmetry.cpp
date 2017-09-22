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
#include <glarot3d/CubeSymmetry.h> 
#include <cassert>
#include <cmath>

namespace glarot3d
{

std::string CubeSymmetry::FACE_NAME[6] = { "x+", "x-", "y+", "y-", "z+", "z-" };
CubeSymmetry::Index CubeSymmetry::SYM_CODES[24] = {   
    5349,   5804,   6490,   6931,  34028,  34469, 
   35155,  35610,  77932,  78373,  80193,  80648, 
  106597, 107052, 108872, 109313, 151635, 152090, 
  152776, 153217, 180314, 180755, 181441, 181896 };
CubeSymmetry::FaceIdx CubeSymmetry::FACE_ADJACENTS[6][4] = {
  { YP, ZP, YN, ZN },
  { ZN, YN, ZP, YP },
  { ZP, XP, ZN, XN },
  { XN, ZN, XP, ZP },
  { XP, YP, XN, YN },
  { YN, XN, YP, XP }
};
CubeSymmetry::Index CubeSymmetry::FACE_DIST[6][6] = {
  { 0, 2, 1, 3, 3, 1 },
  { 2, 0, 3, 1, 1, 2 },
  { 3, 1, 0, 2, 1, 3 },
  { 1, 3, 2, 0, 3, 1 },
  { 1, 3, 3, 1, 0, 2 },
  { 3, 1, 1, 3, 2, 0 }
};

CubeSymmetry::CubeSymmetry() : code_(IDENTITY_CODE) 
{
}

CubeSymmetry::CubeSymmetry(const CubeSymmetry& sym) : code_(sym.code_)
{
}

CubeSymmetry::CubeSymmetry(Index code) : code_(code) 
{
}

CubeSymmetry& CubeSymmetry::operator=(const CubeSymmetry& sym)
{
  code_ = sym.code_;
  return *this;
}

CubeSymmetry::FaceIdx CubeSymmetry::face(FaceIdx f) const
{
  Index i = 3 * (Index)f;
  return (FaceIdx)((code_ >> i) & 7);
}

CubeSymmetry::FaceIdx CubeSymmetry::faceInv(FaceIdx f) const
{
  for (Index i = 0; i < 6; ++i) {
    if ((Index)face(f) == i) return (FaceIdx)i;
  }
  
  return XP;
}

CubeSymmetry::Index CubeSymmetry::distanceFace(FaceIdx f1,FaceIdx f2)
{
  Index i1 = (Index)f1;
  Index i2 = (Index)f2;
  return FACE_DIST[i1][i2];
}

CubeSymmetry::Index CubeSymmetry::distanceFace(Index i1,Index i2)
{
  assert(i1 < 6 && i2 < 6);
  return FACE_DIST[i1][i2];
}

CubeSymmetry::Index CubeSymmetry::faceRotation(FaceIdx f) const
{
  return distanceFace( face(axisU(f)), axisU(face(f)) );
}

bool CubeSymmetry::check(Index code)
{
  for (int i = 0; i < 24; ++i) {
    if (code == SYM_CODES[i]) return true;
  }
  return false;
}

bool CubeSymmetry::check() const
{
  return check(code_);
}

CubeSymmetry CubeSymmetry::rotateX() const
{
  Index val = 0;
  val |= face(XP);
  val |= face(XN) << 3; 
  val |= face(ZP) << 6;
  val |= face(ZN) << 9;
  val |= face(YN) << 12; 
  val |= face(YP) << 15;
  return CubeSymmetry(val);
}

CubeSymmetry CubeSymmetry::rotateX(Index n) const
{
  CubeSymmetry sym(code_);
  Index l = n % 4;
  for (Index i = 0; i < l; ++i) {
    sym = sym.rotateX();
  }
  return sym;
}

CubeSymmetry CubeSymmetry::rotateY() const
{
  Index val = 0;
  val |= face(ZN);
  val |= face(ZP) << 3; 
  val |= face(YP) << 6;
  val |= face(YN) << 9;
  val |= face(XP) << 12; 
  val |= face(XN) << 15;
  return CubeSymmetry(val);
}

CubeSymmetry CubeSymmetry::rotateY(Index n) const
{
  CubeSymmetry sym(code_);
  Index l = n % 4;
  for (Index i = 0; i < l; ++i) {
    sym = sym.rotateY();
  }
  return sym;
}


CubeSymmetry CubeSymmetry::rotateZ() const
{
  Index val = 0;
  val |= face(YN);
  val |= face(YP) << 3; 
  val |= face(XP) << 6;
  val |= face(XN) << 9;
  val |= face(ZP) << 12; 
  val |= face(ZN) << 15;
  return CubeSymmetry(val);
}

CubeSymmetry CubeSymmetry::rotateZ(Index n) const
{
  CubeSymmetry sym(code_);
  Index l = n % 4;
  for (Index i = 0; i < l; ++i) {
    sym = sym.rotateZ();
  }
  return sym;
}

CubeSymmetry CubeSymmetry::compose(const CubeSymmetry& sym) const
{
  Index comp = 0;
  for (Index i = 0; i < 6; ++i) {
//    Index i1 = (Index)face((FaceIdx)i);
//    Index i2 = (Index)sym.face((FaceIdx)i1);
    comp |= (Index)face( sym.face((FaceIdx)i) ) << (3 * i);
//    comp |= i2 << (3 * i);
//    std::cout << FACE_NAME[i] << "(" << i << ") -> " << FACE_NAME[i1] << "(" << i1 << ") -> " << FACE_NAME[i2] << "(" << i2 << ") : " << std::endl;
//    std::cout << "   " << comp << " : " << CubeSymmetry(comp) << std::endl;
  }
  return CubeSymmetry(comp);
}

CubeSymmetry CubeSymmetry::getSymmetry(Index i)
{
  Index m = (i % 24);
  return CubeSymmetry(SYM_CODES[i]);
}

void CubeSymmetry::exportGnuplot(std::ostream& out) const
{
  double faceCenters[6][2] = { { 1.0, 0.0}, {-1.0, 0.0}, { 0.0, 1.0}, { 0.0,-1.0}, { 0.0,  0.0}, { 2.0,  0.0} };  
  double faceDelta[6][6][2] = { 
    { { 0.0, 0.0}, { 0.0, 0.0}, { 0.0, 1.0}, { 0.0,-1.0}, {-1.0, 0.0}, { 1.0, 0.0} },
    { { 0.0, 0.0}, { 0.0, 0.0}, { 0.0, 1.0}, { 0.0,-1.0}, { 1.0, 0.0}, {-1.0, 0.0} },
    { { 1.0, 0.0}, {-1.0, 0.0}, { 0.0, 0.0}, { 0.0, 0.0}, { 0.0,-1.0}, { 0.0, 1.0} },
    { { 1.0, 0.0}, {-1.0, 0.0}, { 0.0, 0.0}, { 0.0, 0.0}, { 0.0, 1.0}, { 0.0,-1.0} },
    { { 1.0, 0.0}, {-1.0, 0.0}, { 0.0, 1.0}, { 0.0,-1.0}, { 0.0, 0.0}, { 0.0, 0.0} },
    { {-1.0, 0.0}, { 1.0, 0.0}, { 0.0, 1.0}, { 0.0,-1.0}, { 0.0, 0.0}, { 0.0, 0.0} }
  };  
  FaceIdx faceAxis[6][2] = { {YP, ZP}, {YN, ZN}, {ZP, XP}, {ZN, XN}, {XP, YP}, {XN, YN} };

  out << "set size ratio -1\n";
  out << "unset key; unset tics; unset border\n";
  out << "set title \"" << (*this) << "\"\n";

  for (int f = 0; f < 6; ++f) {
    Index t = (Index)face((FaceIdx)f);
    std::cout << "face " << f << " " << FACE_NAME[f] << " in face " << t << " " << FACE_NAME[t] 
      << " (" << (faceCenters[t][0]-0.25) << "," << (faceCenters[t][1]-0.40) << ")" << std::endl;
    out << "set label " << (f+1) << " \"" << FACE_NAME[f] << "\" at " << (faceCenters[f][0]-0.45) << "," << (faceCenters[f][1]-0.40) << " tc lt 3\n"
        << "set label " << (f+7) << " \"" << FACE_NAME[f] << "\" at " << (faceCenters[t][0]-0.15) << "," << (faceCenters[t][1]-0.40) << " tc lt 1\n";
  }

  out << "plot ";
  out << "'-' notitle w l lt 9 lw 0.5";
  out << ", '-' notitle w l lt 1 lc 7 lw 2.0";
  out << ", '-' using 1:2:3:4 notitle with vectors lt 1 lc 3 lw 2.0";
  out << ", '-' using 1:2:3:4 notitle with vectors lt 1 lc 1 lw 2.0";
  out << std::endl;

  // Plots the face grids
  for (int f = 0; f < 6; ++f) {
    exportFaceGrids(out,(faceCenters[f][0]-0.5),(faceCenters[f][0]+0.5),(faceCenters[f][1]-0.5),(faceCenters[f][1]+0.5),4);
  }
  out << "e\n";
  
  // Plots opened cube with edges
//  out << (faceCenters[XN][0]-1.0) << " " << (faceCenters[YN][1]-1.0) << "\n"
//      << (faceCenters[ZN][0]+1.0) << " " << (faceCenters[YN][1]-1.0) << "\n"
//      << (faceCenters[ZN][0]+1.0) << " " << (faceCenters[YP][1]+1.0) << "\n"
//      << (faceCenters[XN][0]-1.0) << " " << (faceCenters[YP][1]+1.0) << "\n"
//      << (faceCenters[XN][0]-1.0) << " " << (faceCenters[YN][1]-1.0) << "\n"
//      << std::endl;
  for (Index f = 0; f < 6; ++f) {
    out << (faceCenters[f][0]-0.5) << " " << (faceCenters[f][1]-0.5) << "\n"
        << (faceCenters[f][0]+0.5) << " " << (faceCenters[f][1]-0.5) << "\n"
        << (faceCenters[f][0]+0.5) << " " << (faceCenters[f][1]+0.5) << "\n"
        << (faceCenters[f][0]-0.5) << " " << (faceCenters[f][1]+0.5) << "\n"
        << (faceCenters[f][0]-0.5) << " " << (faceCenters[f][1]-0.5) << "\n"
        << std::endl;
  }
  out << "e" << std::endl;

  // Plots face base axes
  for (Index f = 0; f < 6; ++f) {
    Index v1 = FACE_ADJACENTS[f][0];
    Index v2 = FACE_ADJACENTS[f][1];
    out << faceCenters[f][0] << " " << faceCenters[f][1] << " " << (0.45*faceDelta[f][v1][0]) << " " << (0.45*faceDelta[f][v1][1]) << "\n"
        << faceCenters[f][0] << " " << faceCenters[f][1] << " " << (0.45*faceDelta[f][v2][0]) << " " << (0.45*faceDelta[f][v2][1]) << "\n";
  }
  out << "e" << std::endl;

  // Plots face base axes
  for (Index f = 0; f < 6; ++f) {
    Index t = (Index)face((FaceIdx)f);               // t = face(f)
    Index t1 = (Index)face(FACE_ADJACENTS[f][0]);    // t1 = face( axisU(f) )
    Index t2 = (Index)face(FACE_ADJACENTS[f][1]);    // t2 = face( axisV(f) )
    Index cmp1 = FACE_ADJACENTS[t][0];               // cmp1 = axisU( face(f) )
    out << faceCenters[t][0] << " " << faceCenters[t][1] << " " << (0.25*faceDelta[t][t1][0]) << " " << (0.25*faceDelta[t][t1][1]) << "\n"
        << faceCenters[t][0] << " " << faceCenters[t][1] << " " << (0.25*faceDelta[t][t2][0]) << " " << (0.25*faceDelta[t][t2][1]) << "\n";
    std::cout << FACE_NAME[f] << " (axes " << FACE_NAME[ FACE_ADJACENTS[f][0] ] << "," << FACE_NAME[ FACE_ADJACENTS[f][1] ] << ") -> "
      << FACE_NAME[t] << " (axes " << FACE_NAME[t1] << "," << FACE_NAME[t2] << "): "
      << ": rotated from " << FACE_NAME[t1] << " to " << FACE_NAME[cmp1] << " " << distanceFace(t1,cmp1) << std::endl;
    std::cout 
      << "  axisU(face(" << faceName(f) << ")) = axisU(" << faceName(face(f)) << ") = " << faceName(axisU(face(f))) 
      << ", face(axisU(" << faceName(f) << ")) = face(" << faceName(axisU(f)) << ") = " << faceName(face(axisU(f))) 
      << ", distanceFace( face(axisU(" << faceName(f) << ")), axisU(face(" << faceName(f) << ")) ) = " << distanceFace( face(axisU(f)), axisU(face(f)) ) << std::endl;

//" rot from " << FACE_NAME[FACE_ADJACENTS[f][0]] << " to " << FACE_NAME[t1] << " by " << distanceFace(FACE_ADJACENTS[f][0],t1) << std::endl;
  }
  out << "e" << std::endl;
}

void CubeSymmetry::exportFaceGrids(std::ostream& out,double xmin,double xmax,double ymin,double ymax,int l) const
{
  double dx = (xmax - xmin) / l;
  double dy = (xmax - xmin) / l;

  for (int i = 0; i <= l; ++i) {
    out << "  " << (xmin + dx * i) << " " << ymin << "\n"
        << "  " << (xmin + dx * i) << " " << ymax << "\n\n"
        << "  " << xmin << " " << (ymin + dy * i)  << "\n"
        << "  " << xmax << " " << (ymin + dy * i) << "\n\n";
  }
}

std::ostream& operator<<(std::ostream& out,const CubeSymmetry& sym)
{
  typedef CubeSymmetry::FaceIdx FaceIdx;
  typedef CubeSymmetry::Index Index;
  out << "(";
  for (Index i = 0; i < 6; ++i) {
    out << CubeSymmetry::FACE_NAME[ (Index)sym.face((FaceIdx)i) ];
    if (i < 5) out << ", ";
    else out << ")";
  }
  return out;
}

} // end of namespace

