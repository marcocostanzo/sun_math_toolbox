/*

    Geometry Helper Lib

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

*/

#ifndef GEOMETRY_HELPER_H
#define GEOMETRY_HELPER_H

/*! \file GeometryHelper.h
    \brief Contains helper functions for geometry.
*/

#include "TooN/SVD.h"
#include "TooN/TooN.h"

#define GEOMETRY_HELPER_EPSILON 1.0E-5

namespace sun
{
//! Higer singular value.
/*!
norm(M) is the norm of the Matrix M computed as the
higer singular value.

\param M a matrix.
\return higer singular value of M
*/
double norm(const TooN::Matrix<3>& M);

//! abs element wise
/*!
abs(V) return the vector composed by the absolute values of
the elements of V.

\param v a vector.
*/
TooN::Vector<> abs(const TooN::Vector<>& v);

//! max of a vector
/*!
max(V) return the maximun element of V

\param v a vector.
*/
double max(const TooN::Vector<>& v);

//! min of a vector
/*!
min(V) return the minimum element of V

\param v a vector.
*/
double min(const TooN::Vector<>& v);

//! sum of all elements
/*!
sum(V) is the sum of all the elements of a V

\param v a vector.
*/
double sum(const TooN::Vector<>& v);

//! rotation around x-axis
/*!
rotx(angle) is the SO(3) matrix transformation that rotates
a point around the x-axis for an angle.

\param angle rotation angle.
*/
TooN::Matrix<3, 3> rotx(double angle);

//! rotation around y-axis
/*!
roty(angle) is the SO(3) matrix transformation that rotates
a point around the y-axis for an angle.

\param angle rotation angle.
*/
TooN::Matrix<3, 3> roty(double angle);

//! rotation around z-axis
/*!
rotz(angle) is the SO(3) matrix transformation that rotates
a point around the z-axis for an angle.

\param angle rotation angle.
*/
TooN::Matrix<3, 3> rotz(double angle);

//! signum
/*!
SIGN   Signum function.
   sign(n, zero_out) returns 1 if n
   is greater than zero, zero_out if it equals zero and -1 if it is
   less than zero.

\param n input
\param zero_out output when n=0
*/
double sign(double n, int zero_out);

//! signum
/*!
SIGN   Signum function.
   sign(n) returns 1 if n
   is greater than zero, 0 if it equals zero and -1 if it is
   less than zero.

\param n input
*/
double sign(double n);

//! find the max
/*!
index_max( vector ) return the index (0-based) of
the first occurrence of the greather element in the vector

\param v a vector
*/
int index_max(const TooN::Vector<>& v);

//! pinv DLS
/*!
    Compute pinv as Damped Least Square (DLS)

    \param M a matrix
    \param damping damping factor for the DLS
*/
TooN::Matrix<> pinv_DLS(const TooN::Matrix<>& M, double damping);

//! null space projector
/*!
    Compute a Projector in the null space of M
    This overload takes pinv(M) as second input to improve performance.

    \param M a matrix
    \param M_pinv pinv of M
*/
TooN::Matrix<> nullSpaceProj(const TooN::Matrix<>& M, const TooN::Matrix<>& M_pinv);

//! null space projector
/*!
    Compute a Projector in the null space of M
    This overload internally compute pinv(M) using condition_number.

    \param M a matrix
    \param condition_number for the pinv (default 20)
*/
TooN::Matrix<> nullSpaceProj(const TooN::Matrix<>& M, double condition_number = 20.0);

//! derivate a poly
/*!
    Compute the derivate of a poly

    coeff = [a_n a_n_1 a_n_2 ... a_0]

    p = a_n*x^n + a_n_1*x^n_1 + ... + a_1*x + a_0

    \param coeff vector representing the coefficient of the poly
*/
TooN::Vector<> polydiff(const TooN::Vector<>& coeff);

//! devaluate a poly
/*!
    Evaluate the poly in the point x

    coeff = [a_n a_n_1 a_n_2 ... a_0]

    p = a_n*x^n + a_n_1*x^n_1 + ... + a_1*x + a_0

    \param coeff vector representing the coefficient of the poly
    \param x variable to use to evaluate the poly
*/
double polyval(const TooN::Vector<>& coeff, double x);

}  // namespace sun

#endif