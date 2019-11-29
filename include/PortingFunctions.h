/*

    C++ implementation of some functions from the Matlab Robotic Toolbox by Peter I. Corke.

    Copyright 2018 Università della Campania Luigi Vanvitelli

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

#ifndef ROBOTICS_TOOLBOX_PORTING_FUNCTIONS_H
#define ROBOTICS_TOOLBOX_PORTING_FUNCTIONS_H

/*! \file PortingFunctions.h
    \brief C++ implementation of some functions from the Matlab Robotic Toolbox by Peter I. Corke.
*/

#include "GeometryHelper.h"

#define PORTING_FUNCTIONS_FAIL_COLOR     "\033[1m\033[31m"      /* Bold Red */
#define PORTING_FUNCTIONS_WARN_COLOR     "\033[1m\033[33m"      /* Bold Yellow */
#define PORTING_FUNCTIONS_CRESET         "\033[0m"

//! Create skew-symmetric matrix.
/*!
    S = skew(V) is a skew-symmetric matrix formed from V.

    \f[
    S=
    \left[ {\begin{array}{ccc}
    0  & -v_z &  v_y \\
    v_z &   0 & -v_x \\
    -v_y &  v_x &   0
    \end{array} } \right]
    \f]

    Notes::
        - This is the inverse of the function VEX().
        - These are the generator matrices for the Lie algebras so(2) and so(3).

    References::
        - Robotics, Vision & Control: Second Edition, Chap 2, P. Corke, Springer 2016.

    \param V Input vector [vx vy vz].
    \return The skew-symmetric matrix
    \sa SKEWA(), VEX()
*/
TooN::Matrix<3,3> skew( const TooN::Vector<3>& v );

//! Test if SO(3) rotation matrix.
/*!
    isRot(R) is true (1) if the argument is a rotation matrix, else false (0).

    Notes::
        - A valid rotation matrix has determinant of 1.

    \param m Input matrix to be tested.
    \return true if the argument is a rotation matrix, else false.
    \sa isHomog()
*/
bool isRot(const TooN::Matrix<> &m);

//! Test if SE(3) homogeneous transformation matrix.
/*!
    isHomog(T) is true (1) if the argument T is a Homogeneous transformation matrix, else false (0).

    \param m Input matrix to be tested.
    \return true if the argument is a Homogeneous transformation matrix, else false.
    \sa isRot()
*/
bool isHomog(const TooN::Matrix<> &m);

//! Create an SE(3) translational homogeneous transform.
/*!
    T = transl(X, Y, Z) is an SE(3) homogeneous transform (4x4) representing a pure translation of X, Y and Z.

    \param x Translation in the x direction.
    \param y Translation in the y direction.
    \param z Translation in the z direction.
    \return An SE(3) Homogeneous transformation matrix.
    \sa transl( const TooN::Vector<3>& tr ), transl( const TooN::Matrix<4,4>& T )
*/
TooN::Matrix<4,4> transl( double x, double y, double z );

//! Create an SE(3) translational homogeneous transform.
/*!
    T = transl(P) is an SE(3) homogeneous transform (4x4) representing a translation of P=[X,Y,Z].

    \param tr Translation vector [x y z].
    \return An SE(3) Homogeneous transformation matrix.
    \sa transl( double x, double y, double z ), transl( const TooN::Matrix<4,4>& T )
*/
TooN::Matrix<4,4> transl( const TooN::Vector<3>& tr );

//! Extract the translational part of an SE(3) matrix.
/*!
    P = transl(T) is the translational part of a homogeneous transform T as a 3-element column vector.
    
    This does NOT check if the matrix is SE(3)

    \param T An SE(3) Homogeneous transformation matrix.
    \return The translational part of T.
    \sa transl( double x, double y, double z ), transl( const TooN::Vector<3>& tr )
*/
TooN::Vector<3> transl( const TooN::Matrix<4,4>& T );

//! Rotational submatrix.
/*!
    R = t2r(T) is the orthonormal rotation matrix component of homogeneous transformation matrix T. 
    
    Works for T in SE(3).
    
    Notes::
        - The validity of rotational part is not checked

    \param T An SE(3) Homogeneous transformation matrix.
    \return The rotational part of T.
    \sa r2t, rt2tr
*/
TooN::Matrix<3,3> t2r(const TooN::Matrix<4,4> &T);

//! Convert rotation matrix to a homogeneous transform.
/*!
    T = r2t(R) is or SE(3) homogeneous transform equivalent to an SO(3) orthonormal rotation matrix R with a zero translational component.
    Works for T in SE(3):
    
    Works for T in SE(3).
    
    Notes::
        - Translational component is zero.

    \param R An SO(3) rotation matrix.
    \return The corresponding SE(3) Matrix.
    \sa t2r
*/
TooN::Matrix<4,4> r2t(const TooN::Matrix<3,3> &R);

//! Convert rotation and translation to homogeneous transform.
/*!
    TR = rt2tr(R, t) is a homogeneous transformation matrix (4x4) formed from an orthonormal rotation matrix R (3x3) and a translation vector t (3x1).
    
    Works for R in SO(3).
    
    Notes::
        - The validity of R is not checked

    \param R An SO(3) rotation matrix.
    \return The corresponding SE(3) Matrix.
    \sa t2r, r2t, tr2rt, rt2tr(const TooN::Vector<3> &t, const TooN::Matrix<3,3> &R)
*/
TooN::Matrix<4,4> rt2tr(const TooN::Matrix<3,3> &R, const TooN::Vector<3> &t);

//! Convert rotation and translation to homogeneous transform.
/*!
    TR = rt2tr(t, R) is a homogeneous transformation matrix (4x4) formed from an orthonormal rotation matrix R (3x3) and a translation vector t (3x1).
    
    Works for R in SO(3).
    
    Notes::
        - The validity of R is not checked

    \param R An SO(3) rotation matrix.
    \return The corresponding SE(3) Matrix.
    \sa t2r, r2t, tr2rt, rt2tr(const TooN::Matrix<3,3> &R, const TooN::Vector<3> &t)
*/
TooN::Matrix<4,4> rt2tr(const TooN::Vector<3> &t, const TooN::Matrix<3,3> &R);

//! Rotation Matrix to roll-pitch-yaw angles
/*!
RPY = r2rpy(R) are the roll-pitch-yaw angles (1x3)
corresponding to the rotation matrix R. The 3
angles RPY=[R,P,Y] correspond to sequential rotations about the Z, Y and
X axes respectively. Roll and yaw angles in [-pi,pi) while pitch angle
in [-pi/2,pi/2).

Note::
    - AXES ZYX

\param R a rotation Marix
\return vector [roll, pitch, yaw]
\sa tr2rpy
*/
TooN::Vector<3> r2rpy( const TooN::Matrix<3,3>& R );

//! Homogeneous Transform Matrix to roll-pitch-yaw angles
/*!
RPY = r2rpy(T) are the roll-pitch-yaw angles (1x3)
corresponding to the rotation part of a homogeneous transform T. The 3
angles RPY=[R,P,Y] correspond to sequential rotations about the Z, Y and
X axes respectively. Roll and yaw angles in [-pi,pi) while pitch angle
in [-pi/2,pi/2).

Note::
    - AXES ZYX

\param T a homogeneous transform matrix
\return vector [roll, pitch, yaw]
\sa r2rpy
*/
TooN::Vector<3> tr2rpy( const TooN::Matrix<4,4>& T );

//! Roll-pitch-yaw angles to rotation matrix
/*!
    Compute the rotation matrix equivalent
    to the specified roll, pitch, yaw angles angles. These correspond to
    rotations about the Z, Y, X axes respectively.

    Note::
        - AXES ZYX

    \param roll roll angle [rad]
    \param pitch pitch angle [rad]
    \param yaw yaw angle [rad]
    \return the rotation matrix
    \sa rpy2r, rpy2tr
*/
TooN::Matrix<3,3> rpy2r( double roll, double pitch, double yaw );

//! Roll-pitch-yaw angles to rotation matrix
/*!
    Compute the rotation matrix equivalent
    to the specified roll, pitch, yaw angles angles. These correspond to
    rotations about the Z, Y, X axes respectively.

    Note::
        - AXES ZYX

    \param rpy vector containing [roll pitch yaw] [rad]
    \return the rotation matrix
    \sa rpy2r, rpy2tr
*/
TooN::Matrix<3,3> rpy2r( TooN::Vector<3> &rpy );

//! Roll-pitch-yaw angles to homogeneous transform
/*!
    Compute the homogeneous transformation matrix (4x4) with zero 
    translation and rotation equivalent
    to the specified roll, pitch, yaw angles angles. These correspond to
    rotations about the Z, Y, X axes respectively.

    Note::
        - AXES ZYX

    \param roll roll angle [rad]
    \param pitch pitch angle [rad]
    \param yaw yaw angle [rad]
    \return homogeneous transformation matrix
    \sa rpy2r, rpy2tr
*/
TooN::Matrix<4,4> rpy2tr( double roll, double pitch, double yaw );

//! Roll-pitch-yaw angles to homogeneous transform
/*!
    Compute the homogeneous transformation matrix (4x4) with zero 
    translation and rotation equivalent
    to the specified roll, pitch, yaw angles angles. These correspond to
    rotations about the Z, Y, X axes respectively.

    Note::
        - AXES ZYX

    \param rpy vector containing [roll pitch yaw] [rad]
    \return homogeneous transformation matrix
    \sa rpy2r, rpy2tr
*/
TooN::Matrix<4,4> rpy2tr( TooN::Vector<3> &rpy );

//! Convert rotation matrix to Euler angles.
/*!
    Compute the ZYZ Euler angles (1x3) corresponding to
    the rotation matrix R (3x3). The 3 angles
    EUL=[PHI,THETA,PSI] correspond to sequential rotations about the Z, Y and
    Z axes respectively.

    Notes::
        - AXES ZYZ
        - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
          set to zero and PSI is the sum (PHI+PSI).
        - Translation component is ignored.

    \param R a rotation matrix
    \return a vector containing [PHI,THETA,PSI]
    \sa tr2eul
*/
TooN::Vector<3> r2eul( const TooN::Matrix<3,3>& R );

//! Convert homogeneous transform to Euler angles.
/*!
    Compute the ZYZ Euler angles (1x3) corresponding to
    the rotational part of a homogeneous transform T (4x4). The 3 angles
    EUL=[PHI,THETA,PSI] correspond to sequential rotations about the Z, Y and
    Z axes respectively.

    Notes::
        - AXES ZYZ
        - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
          set to zero and PSI is the sum (PHI+PSI).
        - Translation component is ignored.

    \param T a homogeneous transform matrix
    \return a vector containing [PHI,THETA,PSI]
    \sa tr2eul
*/
TooN::Vector<3> tr2eul( const TooN::Matrix<4,4>& T );

//! Convert Euler angles to rotation matrix
/*!
    Compute the SO(3) rotation matrix equivalent
    to the specified Euler angles. These correspond to rotations about the Z,
    Y, Z axes respectively.

    \param phi angle around Z [rad]
    \param theta angle around Y [rad]
    \param psi angle around Z [rad]
    \return the corresponding rotation matrix
    \sa eul2r, eul2tr
*/
TooN::Matrix<3,3> eul2r( double phi, double theta, double psi );

//! Convert Euler angles to rotation matrix
/*!
    Compute the SO(3) rotation matrix equivalent
    to the specified Euler angles. These correspond to rotations about the Z,
    Y, Z axes respectively.

    \param eul vector containing [phi, theta, psi] [rad]
    \return the corresponding rotation matrix
    \sa eul2r, eul2tr
*/
TooN::Matrix<3,3> eul2r( const TooN::Vector<3> &eul );

//! Convert Euler angles to homogeneous transform
/*!
    Compute the SE(3) homogeneous
    transformation matrix (4x4) with zero translation and rotation equivalent
    to the specified Euler angles. These correspond to rotations about the Z,
    Y, Z axes respectively.

    \param phi angle around Z [rad]
    \param theta angle around Y [rad]
    \param psi angle around Z [rad]
    \return the corresponding homogeneous transform matrix
    \sa eul2r, eul2tr
*/
TooN::Matrix<4,4> eul2tr( double phi, double theta, double psi );

//! Convert Euler angles to homogeneous transform
/*!
    Compute the SE(3) homogeneous
    transformation matrix (4x4) with zero translation and rotation equivalent
    to the specified Euler angles. These correspond to rotations about the Z,
    Y, Z axes respectively.

    \param eul vector containing [phi, theta, psi] [rad]
    \return the corresponding homogeneous transform matrix
    \sa eul2r, eul2tr
*/
TooN::Matrix<4,4> eul2tr( const TooN::Vector<3> &eul );

#endif