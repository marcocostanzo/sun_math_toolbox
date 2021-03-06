/*

    C++ implementation of UnitQuaternion class from the Matlab Robotic Toolbox by Peter I. Corke.

    Copyright 2018-2020 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


    License from the MATLAB files:
    % Copyright (C) 1993-2017, by Peter I. Corke
    %
    % This file is part of The Robotics Toolbox for MATLAB (RTB).
    %
    % RTB is free software: you can redistribute it and/or modify
    % it under the terms of the GNU Lesser General Public License as published by
    % the Free Software Foundation, either version 3 of the License, or
    % (at your option) any later version.
    %
    % RTB is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    % GNU Lesser General Public License for more details.
    %
    % You should have received a copy of the GNU Leser General Public License
    % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
    %
    % http://www.petercorke.com

*/

#ifndef UNIT_QUATERNION_H
#define UNIT_QUATERNION_H

/*! \file UnitQuaternion.h
    \brief C++ implementation of UnitQuaternion class from the Matlab Robotic Toolbox by Peter I. Corke.
*/

#include "sun_math_toolbox/AngVec.h"
#include "sun_math_toolbox/Quaternion.h"

namespace sun
{
//!  UnitQuaternion class to represent a Rotation.
/*!
    See UnitQuaternion.h for more functions.
    \sa Quaternion, UnitQuaternion.h
*/
class UnitQuaternion : public Quaternion
{
private:
protected:
  /*==========Setters=========*/

  /*!
      Set scalar part, this is not allowed for unitQuaternion that must to be UNIT
  */
  virtual void setS(double s);

  /*!
      Set vector part, this is not allowed for unitQuaternion that must to be UNIT
  */
  virtual void setV(const TooN::Vector<3>& v);

  /*==========END Setters=========*/

public:
  /*==========CONSTRUCTORS=========*/

  //! UnitQuaternion from a Vector<4>
  /*!
      UnitQuaternion from a Vector<4>
      Note: if the input vector is not unit it will be normalized and a warning will be printed
  */
  UnitQuaternion(const TooN::Vector<4>& q_vec);

  //! UnitQuaternion from scalar and vector part
  /*!
      UnitQuaternion from scalar and vector part
      Note: if the input is not unit it will be normalized and a warning will be printed
  */
  UnitQuaternion(double s, const TooN::Vector<3>& v);

  //! UnitQuaternion from scalar and vector part
  /*!
      UnitQuaternion from scalar and vector part
      Note: if the input is not unit it will be normalized and a warning will be printed
  */
  UnitQuaternion(const TooN::Vector<3>& v, double s);

  //! Identity UnitQuaternion 1 <0 0 0>
  UnitQuaternion();

  //! UnitQuaternion from Rotation Matrix
  UnitQuaternion(const TooN::Matrix<3, 3>& R);

  //! UnitQuaternion from Rotation Matrix w. continuity
  /*!
      UnitQuaternion from Rotation Matrix preserving continuity with the quaternion oldQ
  */
  UnitQuaternion(const TooN::Matrix<3, 3>& R, const UnitQuaternion& oldQ);

  //! UnitQuaternion from Homogeneous Transformation Matrix
  UnitQuaternion(const TooN::Matrix<4, 4>& T);

  //! UnitQuaternion from Homogeneous Transformation Matrix w. continuity
  /*!
      UnitQuaternion from Homogeneous Transformation Matrix preserving continuity with the quaternion oldQ
  */
  UnitQuaternion(const TooN::Matrix<4, 4>& T, const UnitQuaternion& oldQ);

  //! A copy constructor
  UnitQuaternion(const UnitQuaternion& q) = default;

  //! A copy constructor form the parent class Quaternion
  /*!
      Note: if the input is not unit it will be normalized and a warning will be printed
  */
  UnitQuaternion(const Quaternion& q);

  //! A copy constructor for quaternion Continuity
  /*!
      Copy constructor that preserves continuity with the quaternion oldQ
  */
  UnitQuaternion(const UnitQuaternion& q, const UnitQuaternion& oldQ);

  //! UnitQuaternion from Axis-Angle
  UnitQuaternion(const AngVec& av);

  /*==========END ONSTRUCTORS=========*/

  /*==========VARIE=========*/

  //! Display UnitQuaternion on the std output
  /*!

       Q.display() displays a compact string representation of the quaternion's value
       as a 4-tuple.

       Notes::
       - The vector part is displayed with single brackets < 1, 0, 0 > to
         distinguish it from a Quaternion which displays as << 1, 0, 0 >>

  */
  void display() const;

  //! Preserve continuity
  /*!
      Return the UnitQuaternion equivalent to q that preserve the continuity with oldQ
  */
  static UnitQuaternion continuity(const UnitQuaternion& q, const UnitQuaternion& oldQ);
  /*==========END VARIE=========*/

  /*======UNITQUATERNION FUNCTIONS======*/

  /*
      UnitQuaternion.inv Invert a UnitQuaternion

       QI = Q.inv() is a UnitQuaternion object representing the inverse of Q.

  */
  // !invalid covariant return type!
  // virtual UnitQuaternion inv() const;

  //! Interpolate UnitQuaternions
  /*!
      UnitQuaternion.interp Interpolate UnitQuaternions

       QI = Q.scale(S, OPTIONS) is a UnitQuaternion that interpolates between a null
       rotation (identity quaternion) for S=0 to Q for S=1.

       QI = Q.interp(Q2, S, OPTIONS) as above but interpolates a rotation
       between Q for S=0 and Q2 for S=1.

       Options::
       'shortest'   Take the shortest path along the great circle

       Notes::
       - This is a spherical linear interpolation (slerp) that can be interpretted
         as interpolation along a great circle arc on a sphere.
       - It is an error if S is outside the interval 0 to 1.

       References::
       - Animating rotation with quaternion curves,
         K. Shoemake,
         in Proceedings of ACM SIGGRAPH, (San Francisco), pp. 245-254, 1985.

       See also ctraj.
  */
  UnitQuaternion interp(const UnitQuaternion& q2, double s, bool b_shortest) const;

  //! Interpolate UnitQuaternions
  /*!
      b_shortest = false
      \sa interp
  */
  UnitQuaternion interp(const UnitQuaternion& q2, double s) const;

  //!  Update quaternion by angular displacement
  /*!
      UnitQuaternion.increment Update quaternion by angular displacement

       QU = Q.increment(omega) updates Q by a rotation which is given as a spatial
       displacement omega (3x1) whose direction is the rotation axis and
       magnitude is the amount of rotation.

       See also tr2delta.
  */
  UnitQuaternion increment(const TooN::Vector<3>& w) const;

  // Angle between two UnitQuaternions
  /*!
      UnitQuaternion.angle Angle between two UnitQuaternions

       Q1.theta(Q2) is the angle (in radians) between two UnitQuaternions Q1 and Q2.

       Notes::
       - Either or both Q1 and Q2 can be a vector.

       References::
       - Metrics for 3D rotations: comparison and analysis
         Du Q. Huynh
         J.Math Imaging Vis. DOFI 10.1007/s10851-009-0161-2

       See also Quaternion.angvec.
  */
  double angle(const UnitQuaternion& q2) const;

  //! Quaternion derivative
  /*!
      UnitQuaternion.dot Quaternion derivative

       QD = Q.dot(omega) is the rate of change in the world frame of a body
       frame with attitude Q and angular velocity OMEGA (1x3) expressed as a
       quaternion.

       Notes::
       - This is not a group operator, but it is useful to have the result as a
         quaternion.
       - N.B. The return type is a Quaternion not a UnitQuaternion

       Reference::
        - Robotics, Vision & Control, 2nd edition, Peter Corke, Chap 3.

       See also UnitQuaternion.dotb.
  */
  Quaternion dot(const TooN::Vector<3>& omega) const;

  //! Quaternion derivative IN BODY FRAME
  /*!
      UnitQuaternion.dotb Quaternion derivative IN BODY FRAME

       QD = Q.dotb(omega) is the rate of change in the BODY frame of a body frame
       with attitude Q and angular velocity OMEGA (1x3) expressed as a
       quaternion.

       Notes::
       - This is not a group operator, but it is useful to have the result as a
         quaternion.

       Reference::
        - Robotics, Vision & Control, 2nd edition, Peter Corke, Chap 3.

       See also UnitQuaternion.dot.
  */
  Quaternion dotb(const TooN::Vector<3>& omega) const;

  //! Convert to angle-vector form
  /*!
      UnitQuaternion.angvec Convert to angle-vector form

       [TH,V] = Q.toangvec() as above but also returns a unit vector
       parallel to the rotation axis.

       Notes::
       - Due to the double cover of the quaternion, the returned rotation angles
         will be in the interval [-2pi, 2pi).
  */
  AngVec toangvec() const;

  /*======END UNITQUATERNION FUNCTIONS======*/

  /*======ARITHMETIC OPERATORS======*/

  //! Multiply unit quaternions
  /*!
      UnitQuaternion.mtimes Multiply unit quaternions

       Q1*Q2   is a UnitQuaternion object formed by Hamilton product
       of Q1 and Q2 where Q1 and Q2 are both UnitQuaternion objects.

       Q*V     is a vector (3x1) formed by rotating the vector V (3x1)by the UnitQuaternion Q.

       Notes::
       - Overloaded operator '*'

       See also Quaternion.mrdivide, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
  */
  UnitQuaternion mtimes(const UnitQuaternion& q2) const;

  //! Multiply unit quaternions
  /*!
      \sa mtimes
  */
  TooN::Vector<3> mtimes(const TooN::Vector<3>& v) const;

  //! Multiply unit quaternions
  /*!
      \sa mtimes
  */
  UnitQuaternion operator*(const UnitQuaternion& q2) const;

  //! Multiply unit quaternions
  /*!
      \sa mtimes
  */
  Quaternion operator*(const Quaternion& q2) const;

  //! Multiply unit quaternions
  /*!
      \sa mtimes
  */
  Quaternion operator*(const double& s) const;

  //! Multiply unit quaternions
  /*!
      \sa mtimes
  */
  TooN::Vector<3> operator*(const TooN::Vector<3>& v) const;

  //! Divide unit quaternions
  /*!
      UnitQuaternion.mrdivide Divide unit quaternions

       Q1/Q2   is a UnitQuaternion object formed by Hamilton product of Q1 and
       inv(Q2) where Q1 and Q2 are both UnitQuaternion objects.

       Notes::
       - Overloaded operator '/'
       - If the dividend and divisor are UnitQuaternions, the quotient will be a
         unit quaternion.

       See also Quaternion.mtimes, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
  */
  UnitQuaternion mrdivide(const UnitQuaternion& q2) const;

  //! Divide unit quaternions
  /*!
      \sa mrdivide
  */
  UnitQuaternion operator/(const UnitQuaternion& q2) const;

  //! Divide unit quaternions
  /*!
      \sa mrdivide
  */
  Quaternion operator/(const Quaternion& q2) const;

  //! Divide unit quaternions
  /*!
      \sa mrdivide
  */
  Quaternion operator/(const double& s) const;

  /*!
      See parent Quaternion class
  */
  UnitQuaternion operator^(const int& p) const;

  /*!
      See parent Quaternion class
  */
  Quaternion operator+(const Quaternion& q2) const;

  /*!
      See parent Quaternion class
  */
  UnitQuaternion operator-() const;

  /*!
      See parent Quaternion class
  */
  Quaternion operator-(const Quaternion& q2) const;

  /*======END ARITHMETIC OPERATORS======*/

  /*======TYPE CONVERSION METHODS======*/

  //! ostream
  /*!
      compact string representation of the quaternion's value
      as a 4-tuple.
  */
  friend std::ostream& operator<<(std::ostream& output, const UnitQuaternion& q);
  /*
  operator Quaternion::string() const {
      return string( _s << " < " << _v[0] << ", " << _v[1] << ", " << _v[2] << " >" );
  }
  */

  //! Convert to orthonormal rotation matrix
  /*!
      UnitQuaternion.R Convert to orthonormal rotation matrix

       R = Q.R() is the equivalent SO(3) orthonormal rotation matrix (3x3).

       See also UnitQuaternion.T, UnitQuaternion.SO3.
  */
  TooN::Matrix<3, 3> R() const;

  //! Convert to homogeneous transformation matrix
  /*!
      UnitQuaternion.T Convert to homogeneous transformation matrix

       T = Q.T() is the equivalent SE(3) homogeneous transformation
       matrix (4x4).  If Q is a sequence (Nx1) then T is 4x4xN.

       Notes:
       - Has a zero translational component.

       See also UnitQuaternion.R, UnitQuaternion.SE3.
  */
  TooN::Matrix<4, 4> T() const;

  //! Convert to roll-pitch-yaw angle form.
  /*!
      UnitQuaternion.torpy Convert to roll-pitch-yaw angle form.

       RPY = Q.torpy() are the roll-pitch-yaw angles (1x3) corresponding to
       the UnitQuaternion.  These correspond to rotations about the Z, Y, X axes
       respectively. RPY = [ROLL, PITCH, YAW].

       Options (NOT AVAILABLE)::
        'xyz'   Return solution for sequential rotations about X, Y, Z axes
        'yxz'   Return solution for sequential rotations about Y, X, Z axes

       Notes::
       - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
         set to zero and Y is the sum (R+Y).

       See also UnitQuaternion.toeul, tr2rpy.
  */
  TooN::Vector<3> torpy() const;

  //! Convert to eul angle form.
  /*!
      UnitQuaternion.toeul Convert to eul angle form.

       EUL = Q.toeul(OPTIONS) are the Euler angles (1x3) corresponding to
       the UnitQuaternion.  These correspond to rotations about the Z, Y, Z axes
       respectively. EUL = [PHI,THETA,PSI].

       Options (NOT AVAILABLE)::
        'deg'   Compute angles in degrees (radians default)

       Notes::
       - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
         set to zero and PSI is the sum (PHI+PSI).

       See also UnitQuaternion.toeul, tr2rpy.
  */
  TooN::Vector<3> toeul() const;

  //! Convert to unique 3-vector
  /*!
      UnitQuaternion.tovec Convert to unique 3-vector

       V = Q.tovec() is a vector (1x3) that uniquely represents the UnitQuaternion.  The scalar
       component can be recovered by 1 - norm(V) and will always be positive.

       Notes::
       - UnitQuaternions have double cover of SO(3) so the vector is derived
         from the quaternion with positive scalar component.
       - This vector representation of a UnitQuaternion is used for bundle adjustment.

       See also UnitQuaternion.vec, UnitQuaternion.qvmul.
  */
  TooN::Vector<3> tovec() const;

  //! Test equality of rotation
  /*!
      overloaded version for UnitQuaternions to support double mapping of UnitQuaternion space
  */
  bool eq(const UnitQuaternion& q2) const;

  //! Test equality of rotation
  /*!
      overloaded version for UnitQuaternions to support double mapping of UnitQuaternion space
  */
  bool operator==(const UnitQuaternion& q2) const;

  //! TODO
  bool operator==(const Quaternion& q2) const;

  //! Test inequality of rotation
  /*!
      overloaded version for UnitQuaternions to support double mapping of UnitQuaternion space
  */
  bool operator!=(const UnitQuaternion& q2) const;

  //! TODO
  bool operator!=(const Quaternion& q2) const;

  //! Convert UnitQuaternion to homogeneous transform
  /*!
      TOROT   Convert UnitQuaternion to homogeneous transform

         T = q2tr(Q)

         Return the rotational homogeneous transform corresponding to the unit
         quaternion Q.

         See also: TR2Q
  */
  TooN::Matrix<3, 3> torot() const;

  /*======END TYPE CONVERSION METHODS====*/

  /*=== STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

  //! Construct from rotation about x-axis
  /*!
      UnitQuaternion.Rx Construct from rotation about x-axis

       Q = UnitQuaternion.Rx(ANGLE) is a UnitQuaternion representing rotation of ANGLE about the x-axis.

       See also UnitQuaternion.Ry, UnitQuaternion.Rz.
  */
  static UnitQuaternion Rx(double angle);

  //! Construct from rotation about y-axis
  /*!
      UnitQuaternion.Ry Construct from rotation about y-axis

       Q = UnitQuaternion.Ry(ANGLE) is a UnitQuaternion representing rotation of ANGLE about the y-axis.

       See also UnitQuaternion.Rx, UnitQuaternion.Rz.
  */
  static UnitQuaternion Ry(double angle);

  //! Construct from rotation about z-axis
  /*!
      UnitQuaternion.Rz Construct from rotation about z-axis

       Q = UnitQuaternion.Rz(ANGLE) is a UnitQuaternion representing rotation of ANGLE about the z-axis.

       See also UnitQuaternion.Rx, UnitQuaternion.Ry.
  */
  static UnitQuaternion Rz(double angle);

  //! Construct from angle times rotation vector
  /*!
      UnitQuaternion.omega Construct from angle times rotation vector

       Q = UnitQuaternion.omega(W) is a UnitQuaternion representing rotation of |W| about the vector W (3x1).

       See also UnitQuaternion.angvec.
  */
  static UnitQuaternion omega(const TooN::Vector<3>& w);

  //! Construct from angle times rotation vector
  /*!
      Is an overload of omega

      \sa omega
  */
  static UnitQuaternion omega(double wx, double wy, double wz);

  //! Construct from axos angle
  /*!
      UnitQuaternion.omega Construct from angle and rotation vector

       Q = UnitQuaternion::fromAxisAngle(W) is a UnitQuaternion representing rotation of theta about the vector
     w/norm(w) (3x1).

       See also UnitQuaternion.angvec.
  */
  static UnitQuaternion fromAxisAngle(double theta, const TooN::Vector<3>& w);

  //! Construct from angle and rotation vector
  /*!
      UnitQuaternion.angvec Construct from angle and rotation vector

       Q = UnitQuaternion.angvec(TH, V) is a UnitQuaternion representing rotation of TH about the vector V (3x1).

       See also UnitQuaternion.omega.
  */
  static UnitQuaternion angvec(double theta, const TooN::Vector<3>& v);

  //! Construct from roll-pitch-yaw angles
  /*!
      UnitQuaternion.rpy Construct from roll-pitch-yaw angles

       Q = UnitQuaternion.rpy(ROLL, PITCH, YAW) is a UnitQuaternion
       representing rotation equivalent to the specified roll, pitch, yaw angles
       angles. These correspond to rotations about the Z, Y, X axes
       respectively.

       Options (NOT AVAILABLE)::
       'zyx'   Return solution for sequential rotations about Z, Y, X axes (default)
       'xyz'   Return solution for sequential rotations about X, Y, Z axes
       'yxz'   Return solution for sequential rotations about Y, X, Z axes

       See also UnitQuaternion.eul, rpy2r.
  */
  static UnitQuaternion rpy(double roll, double pitch, double yaw);

  //! Construct from roll-pitch-yaw angles
  /*!
      \sa rpy
  */
  static UnitQuaternion rpy(const TooN::Vector<3>& v_rpy);

  //! Construct from Euler angles
  /*!
      \sa eul
  */
  static UnitQuaternion eul(double phi, double theta, double psi);
  static UnitQuaternion eul(const TooN::Vector<3>& v_eul);

  //! Construct from 3-vector
  /*!
      UnitQuaternion.vec Construct from 3-vector

       Q = UnitQuaternion.vec(V) is a UnitQuaternion constructed from just its vector
       component (1x3) and the scalar part is 1 - norm(V) and will always be positive.

       Notes::
       - This unique and concise vector representation of a UnitQuaternion is used for bundle adjustment.

      See also UnitQuaternion.tovec, UnitVector.qvmul.

  */
  static UnitQuaternion vec(const TooN::Vector<3>& v);

  /*=== END STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

  /*=== OTHER STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

  //! Convert homogeneous transform to a UnitQuaternion
  /*!
      TR2Q   Convert homogeneous transform to a UnitQuaternion

         Q = tr2q(T)

         Return a UnitQuaternion corresponding to the rotational part of the
         homogeneous transform T.

       Reference::
       - Funda, Taylor, IEEE Trans. Robotics and Automation, 6(3), June 1990, pp.
  */
  static UnitQuaternion r2q(const TooN::Matrix<3>& R);

  //! Convert UnitQuaternion to homogeneous transform
  /*!
      TOROT   Convert UnitQuaternion to homogeneous transform

         T = q2tr(Q)

         Return the rotational homogeneous transform corresponding to the unit
         quaternion Q.

         See also: TR2Q
  */
  static TooN::Matrix<3, 3> q2r(const UnitQuaternion& q);

  /*=== END OTHER STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/
};

// OPERATORS

//*CONJ*//
//! Conjugate
UnitQuaternion conj(const UnitQuaternion& q);

//*INV*//
//! Inverse
UnitQuaternion inv(const UnitQuaternion& q);

//*UNIT*//
//! Normalize a Quaternion
UnitQuaternion unit(const Quaternion& q);

//*MTIMES*//
//! Multiply two UnitQuaternion
UnitQuaternion mtimes(const UnitQuaternion& q1, const UnitQuaternion& q2);
// UnitQuaternion operator*(const UnitQuaternion& q1, const UnitQuaternion& q2);

//*MRDIVIDE*//
//! Divide two UnitQuaternion
UnitQuaternion mrdivide(const UnitQuaternion& q1, const UnitQuaternion& q2);
// UnitQuaternion operator/(const UnitQuaternion& q1, const UnitQuaternion& q2);

//*MPOWER*//
//! UnitQuaternion to integer power
UnitQuaternion mpower(const UnitQuaternion& q1, int p);
// UnitQuaternion operator^(const UnitQuaternion& q1, const int& p);

}  // namespace sun

#endif
