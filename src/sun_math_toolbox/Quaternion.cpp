/*

    C++ implementation of Quaternion class from the Matlab Robotic Toolbox by Peter I. Corke.

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

#include "sun_math_toolbox/Quaternion.h"

using namespace TooN;
using namespace std;

namespace sun
{
Quaternion::Quaternion(double s, const Vector<3>& v) : _s(s), _v(v)
{
}

// Zero Quaternion
Quaternion::Quaternion() : Quaternion(0, Zeros)
{
}

Quaternion::Quaternion(const Vector<4>& q_vec) : Quaternion(q_vec[0], q_vec.slice(1, 3))
{
}

/*==========END ONSTRUCTORS=========*/

/*==========Setters=========*/

void Quaternion::setS(double s)
{
  _s = s;
}

void Quaternion::setV(const Vector<3>& v)
{
  _v = v;
}

/*==========END Setters=========*/

/*==========Getters=========*/

double Quaternion::getS() const
{
  return _s;
}

const TooN::Vector<3>& Quaternion::getV() const
{
  return _v;
}

/*==========END Setters=========*/

/*=========VARIE==============*/
/*
    Quaternion.display Display quaternion

     Q.display() displays a compact string representation of the quaternion's value
     as a 4-tuple.

     Notes::
     - The vector part is displayed with double brackets << 1, 0, 0 >> to
       distinguish it from a UnitQuaternion which displays as < 1, 0, 0 >

     See also Quaternion.string.
*/
void Quaternion::display() const
{
  cout << "Quaternion: " << *this << endl;
}

/*=========END VARIE=============*/

/*======QUATERNION FUNCTIONS======*/

/*
    Quaternion.conj Conjugate of a quaternion

     QI = Q.conj() is a quaternion object representing the conjugate of Q.

     Notes::
     - Conjugatation changes the sign of the vector component.

     See also Quaternion.inv.
*/
Quaternion Quaternion::conj() const
{
  return Quaternion(_s, -_v);
}

/*
    Quaternion.inv Invert a quaternion

     QI = Q.inv() is a quaternion object representing the inverse of Q.

     See also Quaternion.conj.
*/
Quaternion Quaternion::inv() const
{
  double n2 = norm2();
  return Quaternion(_s / n2, -_v / n2);
}

/*
    Quaternion.unit Unitize a quaternion

     QU = Q.unit() is a UnitQuaternion object representing the same orientation as Q.

     Notes::
     See also Quaternion.norm, UnitQuaternion.
*/
/* CANNOT BE HERE!
UnitQuaternion Quaternion::unit() const{
    return UnitQuaternion( getDouble()/norm() )
}
*/

/*
    Quaternion.norm Quaternion magnitude

     QN = Q.norm(Q) is the scalar norm or magnitude of the quaternion Q.

     Notes::
     - This is the Euclidean norm of the quaternion written as a 4-vector.
     - A unit-quaternion has a norm of one.

     See also Quaternion.inner, Quaternion.unit.
*/
double Quaternion::norm() const
{
  return TooN::norm(getDouble());
}

/*
    Quaternion.norm2 Quaternion square magnitude

     QN = Q.norm2(Q) is the square of the scalar norm or magnitude of the quaternion Q.

     See also Quaternion.inner, Quaternion.unit.
*/
double Quaternion::norm2() const
{
  return (getDouble() * getDouble());
}

/*
    Quaternion.matrix Matrix representation of Quaternion

     M = Q.matrix() is a matrix (4x4) representation of the Quaternion Q.

     Quaternion, or Hamilton, multiplication can be implemented as a
     matrix-vector product, where the column-vector is the elements of a
     second quaternion:

              matrix(Q1) * double(Q2)'

     Notes::
     - This matrix is not unique, other matrices will serve the purpose for
       multiplication, see https://en.wikipedia.org/wiki/Quaternion#Matrix_representations
     - The determinant of the matrix is the norm of the quaternion to the fourth power.

     See also Quaternion.double, Quaternion.mtimes.
*/
Matrix<4, 4> Quaternion::matrix() const
{
  /*
      Matlab Version:
      m = [q.s    -q.v(1) -q.v(2) -q.v(3)
           q.v(1)  q.s    -q.v(3)  q.v(2)
           q.v(2)  q.v(3)  q.s    -q.v(1)
           q.v(3) -q.v(2)  q.v(1)  q.s];
  */
  return Data(_s, -_v[0], -_v[1], -_v[2], _v[0], _s, -_v[2], _v[1], _v[1], _v[2], _s, -_v[0], _v[2], -_v[1], _v[0], _s);
}

/*
    Quaternion.inner Quaternion inner product

     V = Q1.inner(Q2) is the inner (dot) product of two vectors (1x4),
     comprising the elements of Q1 and Q2 respectively.

     Notes::
     - Q1.inner(Q1) is the same as Q1.norm().

     See also Quaternion.norm.
*/
double Quaternion::inner(const Quaternion& q2) const
{
  return getDouble() * q2.getDouble();
}

/*======END QUATERNION FUNCTIONS======*/

/*======ARITHMETIC OPERATORS======*/

/*
    Quaternion.mtimes Multiply a quaternion object

     Q1*Q2   is a quaternion formed by the Hamilton product of two quaternions.
     Q*S     is the element-wise multiplication of quaternion elements by the scalar S.
     S*Q     is the element-wise multiplication of quaternion elements by the scalar S.

     Notes::
     - Overloaded operator '*'

     See also Quaternion.mrdivide, Quaternion.mpower.
*/
Quaternion Quaternion::mtimes(const Quaternion& q2) const
{
  double s1 = _s;
  double s2 = q2.getS();
  Vector<3> v1 = _v;
  Vector<3> v2 = q2.getV();
  return Quaternion(s1 * s2 - (v1 * v2), s1 * v2 + s2 * v1 + (v1 ^ v2));
}
Quaternion Quaternion::mtimes(double s) const
{
  return Quaternion(getDouble() * s);
}
Quaternion Quaternion::operator*(const Quaternion& q2) const
{
  return mtimes(q2);
}
Quaternion Quaternion::operator*(const double& s) const
{
  return mtimes(s);
}

/*
    Quaternion.mrdivide Quaternion quotient.

     Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
     Q/S     is the element-wise division of quaternion elements by the scalar S.

     Notes::
     - Overloaded operator '/'

     See also Quaternion.mtimes, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
*/
Quaternion Quaternion::mrdivide(const Quaternion& q2) const
{
  return mtimes(sun::inv(q2));
}
Quaternion Quaternion::mrdivide(double s) const
{
  return Quaternion(getDouble() / s);
}
Quaternion Quaternion::operator/(const Quaternion& q2) const
{
  return mrdivide(q2);
}
Quaternion Quaternion::operator/(const double& s) const
{
  return mrdivide(s);
}

/*
    Quaternion.mpower Raise quaternion to integer power

     Q^N is the Quaternion Q raised to the integer power N.

     Notes::
     - Overloaded operator '^'
     - Computed by repeated multiplication.
     - If the argument is a unit-quaternion, the result will be a
       unit quaternion.

     See also Quaternion.mtimes.
*/
Quaternion Quaternion::mpower(int p) const
{
  if (p == 0)
    return Quaternion(1.0, Zeros);

  // multiply by itself so many times
  Quaternion q(*this);
  for (int i = 1; i < ::abs(p); i++)
  {
    q = q * (*this);
  }

  // if exponent was negative, invert it
  if (p < 0)
  {
    q = sun::inv(q);
  }

  return q;
}
Quaternion Quaternion::operator^(const int& p) const
{
  return mpower(p);
}

/*
    PLUS Add quaternions

     Q1+Q2 is a Quaternion formed from the element-wise sum of quaternion elements.

     Notes::
     - Overloaded operator '+'
     - This is not a group operator, but it is useful to have the result as a
       quaternion.

     See also Quaternion.minus.
*/
Quaternion Quaternion::plus(const Quaternion& q2) const
{
  return Quaternion(getDouble() + q2.getDouble());
}
Quaternion Quaternion::operator+(const Quaternion& q2) const
{
  return plus(q2);
}

/*
    Quaternion.minus Subtract quaternions

     Q1-Q2 is a Quaternion formed from the element-wise difference of quaternion elements.

     Notes::
     - Overloaded operator '-'
     - This is not a group operator, but it is useful to have the result as a
       quaternion.

     See also Quaternion.plus.
*/
Quaternion Quaternion::minus(const Quaternion& q2) const
{
  return Quaternion(getDouble() - q2.getDouble());
}
Quaternion Quaternion::operator-() const
{
  return Quaternion(-getDouble());
}
Quaternion Quaternion::operator-(const Quaternion& q2) const
{
  return minus(q2);
}

/*======END ARITHMETIC OPERATORS======*/

/*======RELATIONAL OPERATORS======*/

/*
    ISEQUAL Test quaternion element equality

     ISEQUAL(Q1,Q2) is true if the quaternions Q1 and Q2 are equal.

     Notes::
     - Used by test suite verifyEqual in addition to eq().
     - Invokes eq().

     See also Quaternion.eq.
*/
bool Quaternion::isequal(const Quaternion& q2) const
{
  return eq(q2);
}

/*
    EQ Test quaternion equality

     Q1==Q2 is true if the quaternions Q1 and Q2 are equal.

     Notes::
     - Overloaded operator '=='.
     - This method is invoked for unit Quaternions where Q and -Q represent
       the equivalent rotation, so non-equality does not mean rotations are not
       equivalent.

     See also Quaternion.ne.
*/
bool Quaternion::eq(const Quaternion& q2) const
{
  return (sum(abs(getDouble() - q2.getDouble())) < 100.0 * GEOMETRY_HELPER_EPSILON);
}
bool Quaternion::operator==(const Quaternion& q2) const
{
  return eq(q2);
}

/*
    NE Test quaternion inequality

     Q1 != Q2 is true if the quaternions Q1 and Q2 are not equal.

     Notes::
     - Overloaded operator '!='
     - Note that for unit Quaternions Q and -Q are the equivalent
       rotation, so non-equality does not mean rotations are not
       equivalent.

     See also Quaternion.eq.
*/
bool Quaternion::ne(const Quaternion& q2) const
{
  return !eq(q2);
}
bool Quaternion::operator!=(const Quaternion& q2) const
{
  return ne(q2);
}

/*====== END RELATIONAL OPERATORS======*/

/*======TYPE CONVERSION METHODS======*/

/*
    compact string representation of the quaternion's value
    as a 4-tuple.  If Q is a vector then S has one line per element.
*/
/*
operator Quaternion::string() const {
    return string( _s << " << " << _v[0] << ", " << _v[1] << ", " << _v[2] << " >>" );
}
*/
ostream& operator<<(ostream& output, const Quaternion& q)
{
  output << q._s << " <<" << q._v[0] << ", " << q._v[1] << ", " << q._v[2] << ">>";
  return output;
}

/*
    Quaternion.getDouble Convert a quaternion to a 4-element vector

     V = Q.double() is a row vector (1x4) comprising the quaternion elements,
     scalar then vector.

     elements [s vx vy vz].
*/
Vector<4> Quaternion::getDouble() const
{
  return makeVector(_s, _v[0], _v[1], _v[2]);
}

/*============OPERATORS==========*/

//*CONJ*//
Quaternion conj(const Quaternion& q)
{
  return q.conj();
}
//*INV*//
Quaternion inv(const Quaternion& q)
{
  return q.inv();
}

//*NORM*//
double norm(const Quaternion& q)
{
  return q.norm();
}

//*NORM2*//
double norm2(const Quaternion& q)
{
  return q.norm2();
}

//*MATRIX*//
TooN::Matrix<4, 4> matrix(const Quaternion& q)
{
  return q.matrix();
}

//*INNER*//
double inner(const Quaternion& q1, const Quaternion& q2)
{
  return q1.inner(q2);
}

//*MTIMES*//
Quaternion mtimes(const Quaternion& q1, const Quaternion& q2)
{
  return q1.mtimes(q2);
}
Quaternion mtimes(const Quaternion& q1, double s)
{
  return q1.mtimes(s);
}
Quaternion mtimes(double s, const Quaternion& q2)
{
  return q2.mtimes(s);
} /*
 Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
     return q1.mtimes(q2);
 }
 Quaternion operator*(const Quaternion& q1, const double& s) {
     return q1.mtimes(s);
 }*/
Quaternion operator*(const double& s, const Quaternion& q2)
{
  return q2.mtimes(s);
}

//*MRDIVIDE*//
Quaternion mrdivide(const Quaternion& q1, const Quaternion& q2)
{
  return q1.mrdivide(q2);
}
Quaternion mrdivide(const Quaternion& q1, double s)
{
  return q1.mrdivide(s);
} /*
 Quaternion operator/(const Quaternion& q1, const Quaternion& q2) {
     return q1.mrdivide(q2);
 }
 Quaternion operator/(const Quaternion& q1, const double& s) {
     return q1.mrdivide(s);
 }*/

//*MPOWER*//
Quaternion mpower(const Quaternion& q1, int p)
{
  return q1.mpower(p);
} /*
 Quaternion operator^(const Quaternion& q1, const int& p) {
     return q1.mpower(p);
 }*/

//*PLUS*//
Quaternion plus(const Quaternion& q1, const Quaternion& q2)
{
  return q1.plus(q2);
}
Quaternion plus(const Quaternion& q1, const TooN::Vector<4>& q2_vec)
{
  return q1.plus(q2_vec);
}
Quaternion plus(const TooN::Vector<4>& q1_vec, const Quaternion& q2)
{
  return q2.plus(q1_vec);
} /*
 Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
     return q1.plus(q2);
 }
 Quaternion operator+(const Quaternion& q1, const Vector<4>& q2_vec) {
     return q1.plus(q2_vec);
 }
 Quaternion operator+(const Vector<4>& q1_vec, const Quaternion& q2 ) {
     return q2.plus(q1_vec);
 }*/

//*MINUS*//
Quaternion minus(const Quaternion& q)
{
  return Quaternion(-q.getDouble());
}
Quaternion minus(const Quaternion& q1, const Quaternion& q2)
{
  return q1.minus(q2);
}
Quaternion minus(const Quaternion& q1, const TooN::Vector<4>& q2_vec)
{
  return q1.minus(q2_vec);
}
Quaternion minus(const TooN::Vector<4>& q1_vec, const Quaternion& q2)
{
  return -(q2.minus(q1_vec));
} /*
 Quaternion operator-(const Quaternion& q ) {
     return Quaternion( -q.getDouble() );
 }
 Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
     return q1.minus(q2);
 }
 Quaternion operator-(const Quaternion& q1, const Vector<4>& q2_vec) {
     return q1.minus(q2_vec);
 }
 Quaternion operator-(const Vector<4>& q1_vec, const Quaternion& q2 ) {
     return -q2.minus(q1_vec);
 }*/

//*EQ*//
bool isequal(const Quaternion& q1, const Quaternion& q2)
{
  return q1.eq(q2);
}
bool eq(const Quaternion& q1, const Quaternion& q2)
{
  return q1.eq(q2);
} /*
 bool operator==(const Quaternion& q1, const Quaternion& q2) {
     return q1.eq(q2);
 }*/

//*NE*//
bool ne(const Quaternion& q1, const Quaternion& q2)
{
  return q1.ne(q2);
} /*
 bool operator!=(const Quaternion& q1, const Quaternion& q2) {
     return q1.ne(q2);
 }*/

//*GETDOUBLE*//
TooN::Vector<4> getDouble(const Quaternion& q)
{
  return q.getDouble();
}

}  // namespace sun