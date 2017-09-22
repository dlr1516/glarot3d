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

#include <iostream>

namespace glarot3d
{

/** This class represents the 24 order preserving symmetry in a cube.
 * A symmetry is a permutation of cube faces that preserves ordering. 
 * The internal adopted representation of symmetry uses a code. 
 */
class CubeSymmetry
{
public:
  enum FaceIdx { XP = 0, XN = 1, YP = 2, YN = 3, ZP = 4, ZN = 5 };
  typedef std::uint32_t Index;

  static const Index SYMMETRY_NUM = 24;
  static const Index IDENTITY_CODE = 181896;
  static std::string FACE_NAME[6];
  static Index SYM_CODES[24];
  static FaceIdx FACE_ADJACENTS[6][4];
  static Index FACE_DIST[6][6];

  /** Creates an identity symmetry.
   */
  CubeSymmetry();

  /** Creates a symmetry using its corresponding code. 
   */
  CubeSymmetry(Index rotval);

  /** Creates a copy of a given symmetry.
   */
  CubeSymmetry(const CubeSymmetry& sym);

  /** Assignment operator.
   */
  CubeSymmetry& operator=(const CubeSymmetry& sym);

  /** Returns the internal code.
   */
  Index code() const { return code_; }

  /** Returns the face associated to a given item.
   */
  FaceIdx face(FaceIdx f) const;

  /** Returns the face associated to a given item.
   */
  FaceIdx face(Index f) const { return face((FaceIdx)f); }

  /** Returns the inverse face associated to a given one.
   */
  FaceIdx faceInv(FaceIdx f) const;

  /** Returns the distance between two faces. 
   * Distance is the number k of k*90°rotations to align the two faces.
   */
  static Index distanceFace(FaceIdx f1,FaceIdx f2);

  static Index distanceFace(Index f1,Index f2);

  /** Returns the rotation to be applied on face f in order to be compared 
   * with face face(f). 
   * Computed as: distanceFace( face(axisU(f)), axisU(face(f)) );
   */
  Index faceRotation(FaceIdx f) const;

  /** Returns the rotation to be applied on face f 
   */
  Index faceRotation(Index f) const { return faceRotation((FaceIdx)f); }

  /** Returns true if the symmetry corresponding to the code is correct.
   */
  static bool check(Index code);

  /** Returns true if the code is correct. 
   */
  bool check() const;

  /** Returns the current symmetry 90-degree rotated around face XP (fixed axis!).
   */
  CubeSymmetry rotateX() const;

  /** Returns the current symmetry (90*n)-degree rotated around face XP (fixed axis!).
   */
  CubeSymmetry rotateX(Index n) const;

  /** Returns the current symmetry 90-degree rotated around face YP (fixed axis!).
   */
  CubeSymmetry rotateY() const;

  /** Returns the current symmetry (90*n)-degree rotated around face YP (fixed axis!).
   */
  CubeSymmetry rotateY(Index n) const;

  /** Returns the current symmetry 90-degree rotated around face ZP (fixed axis!).
   */
  CubeSymmetry rotateZ() const;

  /** Returns the current symmetry (90*n)-degree rotated around face ZP (fixed axis!).
   */
  CubeSymmetry rotateZ(Index n) const;

  /** Returns the first axis associates to the given face.
   */
  static FaceIdx axisU(FaceIdx f)
  {
    return FACE_ADJACENTS[(Index)f][0];
  }

  /** Returns the second axis associates to the given face.
   */
  static FaceIdx axisV(FaceIdx f)
  {
    return FACE_ADJACENTS[(Index)f][1];
  }

  /** Returns the first axis associates to the given face.
   */
  static FaceIdx axisU(Index f)
  {
    return FACE_ADJACENTS[f][0];
  }

  /** Returns the second axis associates to the given face.
   */
  static FaceIdx axisV(Index f)
  {
    return FACE_ADJACENTS[f][1];
  }

  /** Returns the string name of a face.
   */
  static std::string faceName(FaceIdx f) { return std::string(FACE_NAME[(Index)f]); }

  /** Returns the string name of a face given its index.
   */
  static std::string faceName(Index f) { return std::string(FACE_NAME[f]); }

  /** Composes this symmetry with the given one and returns the result. 
   */
  CubeSymmetry compose(const CubeSymmetry& sym) const;
  
  /** Returns one of the 24 symmetries.
   */
  static CubeSymmetry getSymmetry(Index i);

  /** Prints the gnuplot commands to plot cubemap-like view of rotated.
   */
  void exportGnuplot(std::ostream& out) const;

private:
  Index code_;

  void exportFaceGrids(std::ostream& out,double xmin,double xmax,double ymin,double ymax,int l) const;
};

/** Prints the cube symmetry. 
 */
std::ostream& operator<<(std::ostream& out,const CubeSymmetry& sym);

} // end of namespace glarot3d


