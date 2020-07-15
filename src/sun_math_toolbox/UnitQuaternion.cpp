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

#include "sun_math_toolbox/UnitQuaternion.h"

using namespace TooN;
using namespace std;

namespace sun
{
// Private do nothing
void UnitQuaternion::setS(double s)
{
}

void UnitQuaternion::setV(const TooN::Vector<3>& v)
{
}

/*==========CONSTRUCTORS=========*/

UnitQuaternion::UnitQuaternion(const Vector<4>& q_vec)
{
  double n = ::norm(q_vec);
  Vector<4> q_vec_in = q_vec;
  if (fabs(n - 1.0) > 10.0 * GEOMETRY_HELPER_EPSILON)
  {
    cout << PORTING_FUNCTIONS_WARN_COLOR "[UnitQuaternion] UnitQuaternion( const Vector<4>& q_vec ): input Vector is "
                                         "not Unit! i will do an automatic normalization..." PORTING_FUNCTIONS_CRESET
         << endl;
    q_vec_in = q_vec / n;
  }
  _s = q_vec_in[0];
  _v = q_vec_in.slice(1, 3);
}

UnitQuaternion::UnitQuaternion(double s, const Vector<3>& v) : UnitQuaternion(makeVector(s, v[0], v[1], v[2]))
{
}

UnitQuaternion::UnitQuaternion(const Vector<3>& v, double s) : UnitQuaternion(makeVector(s, v[0], v[1], v[2]))
{
}

// Identity Quaternion 1 <0 0 0>
UnitQuaternion::UnitQuaternion()
{
  _s = 1.0;
  _v = Zeros;
}

// From Rotation Matrix
UnitQuaternion::UnitQuaternion(const Matrix<3, 3>& R)
{
  UnitQuaternion uq = UnitQuaternion::r2q(R);
  _s = uq.getS();
  _v = uq.getV();
}

UnitQuaternion::UnitQuaternion(const Matrix<3, 3>& R, const UnitQuaternion& oldQ)
  : UnitQuaternion(continuity(UnitQuaternion(R), oldQ))
{
}

// From Homogeneous Transform
UnitQuaternion::UnitQuaternion(const Matrix<4, 4>& T) : UnitQuaternion(Matrix<3, 3>(T.slice<0, 0, 3, 3>()))
{
}

UnitQuaternion::UnitQuaternion(const Matrix<4, 4>& T, const UnitQuaternion& oldQ)
  : UnitQuaternion(continuity(UnitQuaternion(T), oldQ))
{
}

UnitQuaternion::UnitQuaternion(const Quaternion& q)
{
  // Check if is unit
  double n = q.norm();
  if (fabs(n - 1.0) > 10.0 * GEOMETRY_HELPER_EPSILON)
  {
    cout << PORTING_FUNCTIONS_WARN_COLOR "[UnitQuaternion] WARN in UnitQuaternion( const Quaternion& q ): input "
                                         "Quaternion is not Unit! i will do an automatic normalization... "
         << "Maybe you want to use UnitQuaternion unit(const Quaternion&)" PORTING_FUNCTIONS_CRESET << endl;
  }
  Vector<4> uq = q.getDouble() / n;
  _s = uq[0];
  _v = uq.slice<1, 3>();
}

// A copy constructor for quaternion Continuity
UnitQuaternion::UnitQuaternion(const UnitQuaternion& q, const UnitQuaternion& oldQ)
  : UnitQuaternion(continuity(q, oldQ))
{
}

// From Axis Angle
UnitQuaternion::UnitQuaternion(const AngVec& av)
{
  UnitQuaternion uq = UnitQuaternion::angvec(av.getAng(), av.getVec());
  _s = uq.getS();
  _v = uq.getV();
}

/*==========END ONSTRUCTORS=========*/

/*==========VARIE=========*/
/*
    UnitQuaternion.display Display unitquaternion

     Q.display() displays a compact string representation of the unit quaternion's value
     as a 4-tuple.

     Notes::
     - The vector part is displayed with single brackets < 1, 0, 0 > to
       distinguish it from a Quaternion which displays as << 1, 0, 0 >>

     See also Quaternion.string.
*/
void UnitQuaternion::display() const
{
  cout << "UnitQuaternion: " << *this << endl;
}

UnitQuaternion UnitQuaternion::continuity(const UnitQuaternion& q, const UnitQuaternion& oldQ)
{
  if ((q.getV() * oldQ.getV()) < -0.01)
  {
    return (-q);
  }
  return q;
}

/*==========END VARIE=========*/

/*======UNITQUATERNION FUNCTIONS======*/

/*
    UnitQuaternion.inv Invert a UnitQuaternion

     QI = Q.inv() is a UnitQuaternion object representing the inverse of Q.

*/
// !invalid covariant return type!
/*
UnitQuaternion UnitQuaternion::inv() const{
    return UnitQuaternion( _s , -_v );
}
*/

/*
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
UnitQuaternion UnitQuaternion::interp(const UnitQuaternion& q2, double s, bool b_shortest) const
{
  Vector<4> q1_vec = getDouble();
  Vector<4> q2_vec = q2.getDouble();

  double cosTheta = q1_vec * q2_vec;

  if (b_shortest)
  {
    // take shortest path along the great circle, patch by Gauthier Gras
    if (cosTheta < 0)
    {
      q1_vec = -q1_vec;
      cosTheta = -cosTheta;
    }
  }

  double theta = acos(cosTheta);

  // check s range
  if (s < 0 || s > 1)
  {
    cout << PORTING_FUNCTIONS_FAIL_COLOR "[UnitQuaternion]: error in interp() | s must be in [0 1] | s = " << s
         << PORTING_FUNCTIONS_CRESET << endl;
    exit(-1);
  }

  if (theta == 0)
    return UnitQuaternion(q1_vec);

  return UnitQuaternion((sin((1 - s) * theta) * q1_vec + sin(s * theta) * q2_vec) / sin(theta));
}

UnitQuaternion UnitQuaternion::interp(const UnitQuaternion& q2, double s) const
{
  return interp(q2, s, false);
}

/*
    UnitQuaternion.increment Update quaternion by angular displacement

     QU = Q.increment(omega) updates Q by a rotation which is given as a spatial
     displacement omega (3x1) whose direction is the rotation axis and
     magnitude is the amount of rotation.

     See also tr2delta.
*/
UnitQuaternion UnitQuaternion::increment(const Vector<3>& w) const
{
  return UnitQuaternion(mtimes(UnitQuaternion::omega(w)));
}

/*
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
double UnitQuaternion::angle(const UnitQuaternion& q2) const
{
  Vector<4> q1_vec = getDouble();
  Vector<4> q2_vec = q2.getDouble();
  // use 2 atan2(|q1-q2|, |q1+q2|)
  return 2.0 * atan2(::norm(q1_vec - q2_vec), ::norm(q1_vec + q2_vec));
}

/*
    UnitQuaternion.dot Quaternion derivative

     QD = Q.dot(omega) is the rate of change in the world frame of a body
     frame with attitude Q and angular velocity OMEGA (1x3) expressed as a
     quaternion.

     Notes::
     - This is not a group operator, but it is useful to have the result as a
       quaternion.

     Reference::
      - Robotics, Vision & Control, 2nd edition, Peter Corke, Chap 3.

     See also UnitQuaternion.dotb.
*/
// The return type is not UnitQuaternion
Quaternion UnitQuaternion::dot(const Vector<3>& omega) const
{
  // UnitQuaternion.pure(omega) * (*this)
  Matrix<3, 3> E = _s * Identity(3) - skew(_v);
  return Quaternion(-0.5 * _v * omega, 0.5 * E * omega);
}

/*
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
Quaternion UnitQuaternion::dotb(const Vector<3>& omega) const
{
  // q * UnitQuaternion.pure(omega)
  Matrix<3, 3> E = _s * Identity(3) + skew(_v);  // the difference is the PLUS!! +++
  return Quaternion(-0.5 * _v * omega, 0.5 * E * omega);
}

/*
    UnitQuaternion.angvec Convert to angle-vector form

     [TH,V] = Q.toangvec() as above but also returns a unit vector
     parallel to the rotation axis.

     Notes::
     - Due to the double cover of the quaternion, the returned rotation angles
       will be in the interval [-2pi, 2pi).
*/

AngVec UnitQuaternion::toangvec() const
{
  if ( ::norm(_v) < 10.0 * GEOMETRY_HELPER_EPSILON)
  {
    return AngVec();
  }

  return AngVec(2.0 * atan2(TooN::norm(_v), _s), unit(_v));
}

/*======END UNITQUATERNION FUNCTIONS======*/

/*======ARITHMETIC OPERATORS======*/

/*
    UnitQuaternion.mtimes Multiply unit quaternions

     Q1*Q2   is a UnitQuaternion object formed by Hamilton product
     of Q1 and Q2 where Q1 and Q2 are both UnitQuaternion objects.

     Q*V     is a vector (3x1) formed by rotating the vector V (3x1)by the UnitQuaternion Q.

     Notes::
     - Overloaded operator '*'

     See also Quaternion.mrdivide, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
*/

UnitQuaternion UnitQuaternion::mtimes(const UnitQuaternion& q2) const
{
  return UnitQuaternion(Quaternion::mtimes(q2));
}
Vector<3> UnitQuaternion::mtimes(const Vector<3>& v) const
{
  Quaternion q = (Quaternion::mtimes(Quaternion(0.0, v))) * sun::inv(*this);
  return q.getV();
}
UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion& q2) const
{
  return mtimes(q2);
}
Quaternion UnitQuaternion::operator*(const Quaternion& q2) const
{
  return Quaternion::mtimes(q2);
}
Quaternion UnitQuaternion::operator*(const double& s) const
{
  return Quaternion::mtimes(s);
}
TooN::Vector<3> UnitQuaternion::operator*(const TooN::Vector<3>& v) const
{
  return mtimes(v);
}

/*
    UnitQuaternion.mrdivide Divide unit quaternions

     Q1/Q2   is a UnitQuaternion object formed by Hamilton product of Q1 and
     inv(Q2) where Q1 and Q2 are both UnitQuaternion objects.

     Notes::
     - Overloaded operator '/'
     - If the dividend and divisor are UnitQuaternions, the quotient will be a
       unit quaternion.

     See also Quaternion.mtimes, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
*/
UnitQuaternion UnitQuaternion::mrdivide(const UnitQuaternion& q2) const
{
  return mtimes(sun::inv(q2));
}
UnitQuaternion UnitQuaternion::operator/(const UnitQuaternion& q2) const
{
  return mrdivide(q2);
}
Quaternion UnitQuaternion::operator/(const Quaternion& q2) const
{
  return Quaternion::mrdivide(q2);
}
Quaternion UnitQuaternion::operator/(const double& s) const
{
  return Quaternion::mrdivide(s);
}

UnitQuaternion UnitQuaternion::operator^(const int& p) const
{
  return UnitQuaternion(mpower(p));
}
Quaternion UnitQuaternion::operator+(const Quaternion& q2) const
{
  return plus(q2);
}
UnitQuaternion UnitQuaternion::operator-() const
{
  return UnitQuaternion(-getDouble());
}
Quaternion UnitQuaternion::operator-(const Quaternion& q2) const
{
  return minus(q2);
}

/*======END ARITHMETIC OPERATORS======*/

/*======TYPE CONVERSION METHODS======*/

/*
    compact string representation of the quaternion's value
    as a 4-tuple.  If Q is a vector then S has one line per element.
*/
/*
operator Quaternion::string() const {
    return string( _s << " < " << _v[0] << ", " << _v[1] << ", " << _v[2] << " >" );
}
*/
ostream& operator<<(ostream& output, const UnitQuaternion& q)
{
  output << q._s << " <" << q._v[0] << ", " << q._v[1] << ", " << q._v[2] << ">";
  return output;
}

/*
    UnitQuaternion.R Convert to orthonormal rotation matrix

     R = Q.R() is the equivalent SO(3) orthonormal rotation matrix (3x3).

     See also UnitQuaternion.T, UnitQuaternion.SO3.
*/

Matrix<3, 3> UnitQuaternion::R() const
{
  return torot();
}

/*
    UnitQuaternion.T Convert to homogeneous transformation matrix

     T = Q.T() is the equivalent SE(3) homogeneous transformation
     matrix (4x4).  If Q is a sequence (Nx1) then T is 4x4xN.

     Notes:
     - Has a zero translational component.

     See also UnitQuaternion.R, UnitQuaternion.SE3.
*/

Matrix<4, 4> UnitQuaternion::T() const
{
  Matrix<4, 4> out = transl(Vector<3>(Zeros));
  out.slice(0, 0, 3, 3) = torot();
  return out;
}

/*
    UnitQuaternion.torpy Convert to roll-pitch-yaw angle form.

     RPY = Q.torpy() are the roll-pitch-yaw angles (1x3) corresponding to
     the UnitQuaternion.  These correspond to rotations about the Z, Y, X axes
     respectively. RPY = [ROLL, PITCH, YAW].

     Notes::
     - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
       set to zero and Y is the sum (R+Y).

     See also UnitQuaternion.toeul, tr2rpy.
*/
Vector<3> UnitQuaternion::torpy() const
{
  return r2rpy(torot());
}

/*
    UnitQuaternion.toeul Convert to roll-pitch-yaw angle form.

     EUL = Q.toeul(OPTIONS) are the Euler angles (1x3) corresponding to
     the UnitQuaternion.  These correspond to rotations about the Z, Y, Z axes
     respectively. EUL = [PHI,THETA,PSI].

     Options::
      'deg'   Compute angles in degrees (radians default)

     Notes::
     - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
       set to zero and PSI is the sum (PHI+PSI).

     See also UnitQuaternion.toeul, tr2rpy.
*/
Vector<3> UnitQuaternion::toeul() const
{
  return r2eul(torot());
}

/*
    UnitQuaternion.tovec Convert to unique 3-vector

     V = Q.tovec() is a vector (1x3) that uniquely represents the UnitQuaternion.  The scalar
     component can be recovered by 1 - norm(V) and will always be positive.

     Notes::
     - UnitQuaternions have double cover of SO(3) so the vector is derived
       from the quaternion with positive scalar component.
     - This vector representation of a UnitQuaternion is used for bundle adjustment.

     See also UnitQuaternion.vec, UnitQuaternion.qvmul.
*/
Vector<3> UnitQuaternion::tovec() const
{
  if (_s < 0.0)
  {
    return -_v;
  }
  else
  {
    return _v;
  }
}

/*
    overloaded version for UnitQuaternions to support double mapping
*/
bool UnitQuaternion::eq(const UnitQuaternion& q2) const
{
  Vector<4> q1_vec = getDouble();
  Vector<4> q2_vec = q2.getDouble();
  return (sum(abs(q1_vec - q2_vec)) < 100.0 * GEOMETRY_HELPER_EPSILON) ||
         (sum(abs(q1_vec + q2_vec)) < 100.0 * GEOMETRY_HELPER_EPSILON);
}

bool UnitQuaternion::operator==(const UnitQuaternion& q2) const
{
  return eq(q2);
}
bool UnitQuaternion::operator==(const Quaternion& q2) const
{
  return eq(q2);
}

bool UnitQuaternion::operator!=(const UnitQuaternion& q2) const
{
  return !eq(q2);
}
bool UnitQuaternion::operator!=(const Quaternion& q2) const
{
  return ne(q2);
}

/*
    TOROT   Convert UnitQuaternion to homogeneous transform

       T = q2tr(Q)

       Return the rotational homogeneous transform corresponding to the unit
       quaternion Q.

       See also: TR2Q
*/
Matrix<3, 3> UnitQuaternion::torot() const
{
  Vector<4> q_vec = getDouble();
  double s = q_vec[0];
  double x = q_vec[1];
  double y = q_vec[2];
  double z = q_vec[3];

  return Data(

      1.0 - 2.0 * (pow(y, 2) + pow(z, 2)), 2.0 * (x * y - s * z), 2.0 * (x * z + s * y), 2.0 * (x * y + s * z),
      1.0 - 2.0 * (pow(x, 2) + pow(z, 2)), 2.0 * (y * z - s * x), 2.0 * (x * z - s * y), 2.0 * (y * z + s * x),
      1.0 - 2.0 * (pow(x, 2) + pow(y, 2))

  );
}

/*======END TYPE CONVERSION METHODS====*/

/*=== STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

/*
    UnitQuaternion.Rx Construct from rotation about x-axis

     Q = UnitQuaternion.Rx(ANGLE) is a UnitQuaternion representing rotation of ANGLE about the x-axis.

     See also UnitQuaternion.Ry, UnitQuaternion.Rz.
*/
UnitQuaternion UnitQuaternion::Rx(double angle)
{
  return UnitQuaternion(rotx(angle));
}

/*
    UnitQuaternion.Ry Construct from rotation about y-axis

     Q = UnitQuaternion.Ry(ANGLE) is a UnitQuaternion representing rotation of ANGLE about the y-axis.

     See also UnitQuaternion.Rx, UnitQuaternion.Rz.
*/
UnitQuaternion UnitQuaternion::Ry(double angle)
{
  return UnitQuaternion(roty(angle));
}

/*
    UnitQuaternion.Rz Construct from rotation about z-axis

     Q = UnitQuaternion.Rz(ANGLE) is a UnitQuaternion representing rotation of ANGLE about the z-axis.

     See also UnitQuaternion.Rx, UnitQuaternion.Ry.
*/
UnitQuaternion UnitQuaternion::Rz(double angle)
{
  return UnitQuaternion(rotz(angle));
}

/*
    UnitQuaternion.omega Construct from angle times rotation vector

     Q = UnitQuaternion.omega(W) is a UnitQuaternion representing rotation of |W| about the vector W (3x1).

     See also UnitQuaternion.angvec.
*/
UnitQuaternion UnitQuaternion::omega(const Vector<3>& w)
{
  double theta = TooN::norm(w);
  double s = cos(theta / 2.0);
  Vector<3> v = sin(theta / 2.0) * unit(w);
  return UnitQuaternion(s, v);
}
UnitQuaternion UnitQuaternion::omega(double wx, double wy, double wz)
{
  return UnitQuaternion::omega(makeVector(wx, wy, wz));
}
UnitQuaternion UnitQuaternion::fromAxisAngle(double theta, const Vector<3>& w)
{
  return UnitQuaternion::omega(unit(w) * theta);
}

/*
    UnitQuaternion.angvec Construct from angle and rotation vector

     Q = UnitQuaternion.angvec(TH, V) is a UnitQuaternion representing rotation of TH about the vector V (3x1).

     See also UnitQuaternion.omega.
*/
UnitQuaternion UnitQuaternion::angvec(double theta, const Vector<3>& v)
{
  return UnitQuaternion(cos(theta / 2.0), sin(theta / 2.0) * unit(v));
}

/*
    UnitQuaternion.rpy Construct from roll-pitch-yaw angles

     Q = UnitQuaternion.rpy(ROLL, PITCH, YAW) is a UnitQuaternion
     representing rotation equivalent to the specified roll, pitch, yaw angles
     angles. These correspond to rotations about the Z, Y, X axes
     respectively.

     Options::
     'zyx'   Return solution for sequential rotations about Z, Y, X axes (default)
     'xyz'   Return solution for sequential rotations about X, Y, Z axes
     'yxz'   Return solution for sequential rotations about Y, X, Z axes

     See also UnitQuaternion.eul, rpy2r.
*/
UnitQuaternion UnitQuaternion::rpy(double roll, double pitch, double yaw)
{
  return UnitQuaternion(rpy2r(roll, pitch, yaw));
}
UnitQuaternion UnitQuaternion::rpy(const Vector<3>& v_rpy)
{
  return UnitQuaternion::rpy(v_rpy[0], v_rpy[1], v_rpy[2]);
}

/*
    UnitQuaternion.eul Construct from Euler angles

     Q = UnitQuaternion.eul(PHI, THETA, PSI) is a UnitQuaternion
     representing rotation equivalent to the specified Euler angles
     angles. These correspond to rotations about the Z, Y, Z axes
     respectively.
*/
UnitQuaternion UnitQuaternion::eul(double phi, double theta, double psi)
{
  return UnitQuaternion(eul2r(phi, theta, psi));
}
UnitQuaternion UnitQuaternion::eul(const Vector<3>& v_eul)
{
  return UnitQuaternion::eul(v_eul[0], v_eul[1], v_eul[2]);
}

/*
    UnitQuaternion.vec Construct from 3-vector

     Q = UnitQuaternion.vec(V) is a UnitQuaternion constructed from just its vector
     component (1x3) and the scalar part is 1 - norm(V) and will always be positive.

     Notes::
     - This unique and concise vector representation of a UnitQuaternion is used for bundle adjustment.

    See also UnitQuaternion.tovec, UnitVector.qvmul.

*/
UnitQuaternion UnitQuaternion::vec(const Vector<3>& v)
{
  return UnitQuaternion(1.0 - TooN::norm(v), v);
}

/*=== END STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

/*=== OTHER STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

/*
    TR2Q   Convert homogeneous transform to a UnitQuaternion

       Q = tr2q(T)

       Return a UnitQuaternion corresponding to the rotational part of the
       homogeneous transform T.

     Reference::
     - Funda, Taylor, IEEE Trans. Robotics and Automation, 6(3), June 1990, pp.
*/
UnitQuaternion UnitQuaternion::r2q(const Matrix<3>& R)
{
  // Check matrix
  if (!isRot(R))
  {
    throw std::invalid_argument("[UnitQuaternion] Error in r2q( const Matrix<3,3>& R ): R is not a rotation Matrix");
  }

  // Using modified version of sign() function as per the paper
  //  sign(x) = 1 if x>=0

  double trace_1 = trace(R) + 1.0;
  if (trace_1 < 0.0)
  {
    if (trace_1 > -10.0 * std::numeric_limits<double>::epsilon())
    {
      trace_1 = 0.0;  //<- Numeric fix
    }
    else
    {
      // trace+1 is negative for another reason
      throw std::logic_error("[UnitQuaternion] Error in r2q( const Matrix<3,3>& R ): trace+1 of R is negative. Please "
                             "contact the mantainer.");
    }
  }
  double s = sqrt(trace_1) / 2.0;
  double kx = R[2][1] - R[1][2];  // Oz - Ay
  double ky = R[0][2] - R[2][0];  // Ax - Nz
  double kz = R[1][0] - R[0][1];  // Ny - Ox

  // for the numerical case, deal with rotations by very small angles

  // equation (7)
  int k = index_max(R.diagonal_slice());
  double kx1, ky1, kz1;
  double sgn;
  switch (k)
  {
    case 0:
    {                                           // Nx dominates
      kx1 = R[0][0] - R[1][1] - R[2][2] + 1.0;  // Nx - Oy - Az + 1
      ky1 = R[1][0] + R[0][1];                  // Ny + Ox
      kz1 = R[2][0] + R[0][2];                  // Nz + Ax
      sgn = sign(kx, 1);
      break;
    }
    case 1:
    {                                           // Oy dominates
      kx1 = R[1][0] + R[0][1];                  // Ny + Ox
      ky1 = R[1][1] - R[0][0] - R[2][2] + 1.0;  // Oy - Nx - Az + 1
      kz1 = R[2][1] + R[1][2];                  // Oz + Ay
      sgn = sign(ky, 1);
      break;
    }
    case 2:
    {                                           // Az dominates
      kx1 = R[2][0] + R[0][2];                  // Nz + Ax
      ky1 = R[2][1] + R[1][2];                  // Oz + Ay
      kz1 = R[2][2] - R[0][0] - R[1][1] + 1.0;  // Az - Nx - Oy + 1
      sgn = sign(kz, 1);
      break;
    }
    default:
    {
      /* Default Code */
      throw std::logic_error("[UnitQuaternion] Error in r2q( const Matrix<3,3>& R ): invalid index in index_max(). "
                             "Please contact the mantainer.");
    }
  }

  // equation (8)
  kx = kx + sgn * kx1;
  ky = ky + sgn * ky1;
  kz = kz + sgn * kz1;

  Vector<3> k_vec = makeVector(kx, ky, kz);
  double nm = TooN::norm(k_vec);
  if (nm == 0)
  {
    // handle special case of null quaternion
    return UnitQuaternion();
  }
  else
  {
    return UnitQuaternion(s, k_vec * sqrt(1.0 - pow(s, 2)) / nm);
  }
}

/*
    TOROT   Convert UnitQuaternion to homogeneous transform

       T = q2tr(Q)

       Return the rotational homogeneous transform corresponding to the unit
       quaternion Q.

       See also: TR2Q
*/
Matrix<3, 3> UnitQuaternion::q2r(const UnitQuaternion& q)
{
  return q.torot();
}

/*=== END OTHER STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS ==*/

// OPERATORS

//*CONJ*//
UnitQuaternion conj(const UnitQuaternion& q)
{
  return UnitQuaternion(q.conj());
}

//*INV*//
UnitQuaternion inv(const UnitQuaternion& q)
{
  return UnitQuaternion(q.getS(), -q.getV());
}

//*UNIT*//
UnitQuaternion unit(const Quaternion& q)
{
  return UnitQuaternion(q / norm(q));
}

//*MTIMES*//
UnitQuaternion mtimes(const UnitQuaternion& q1, const UnitQuaternion& q2)
{
  return q1.mtimes(q2);
} /*
 UnitQuaternion operator*(const UnitQuaternion& q1, const UnitQuaternion& q2){
     return q1.mtimes(q2);
 }*/

//*MRDIVIDE*//
UnitQuaternion mrdivide(const UnitQuaternion& q1, const UnitQuaternion& q2)
{
  return q1.mrdivide(q2);
}
// UnitQuaternion operator/(const UnitQuaternion& q1, const UnitQuaternion& q2);

//*MPOWER*//
UnitQuaternion mpower(const UnitQuaternion& q1, int p)
{
  return UnitQuaternion(q1.mpower(p));
}
// UnitQuaternion operator^(const UnitQuaternion& q1, const int& p);

}  // namespace sun