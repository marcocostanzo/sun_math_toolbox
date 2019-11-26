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

/*
%TRANSL Create an SE(3) translational homogeneous transform
%
% Create a translational SE(3) matrix::
%
% T = TRANSL(X, Y, Z) is an SE(3) homogeneous transform (4x4) representing
% a pure translation of X, Y and Z.
%
% T = TRANSL(P) is an SE(3) homogeneous transform (4x4) representing a
% translation of P=[X,Y,Z].
%
% Extract the translational part of an SE(3) matrix::
%
% P = TRANSL(T) is the translational part of a homogeneous transform T as a
% 3-element column vector. This does NOT check if the matrix is SE(3)
%
% Notes::
% - Somewhat unusually this function performs a function and its inverse.  An
%   historical anomaly.
*/
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

/*!
%TR2RPY/R2RPY Convert a homogeneous transform / Rotation Matrix to roll-pitch-yaw angles
%
% RPY = TR2RPY(T) are the roll-pitch-yaw angles (1x3)
% corresponding to the rotation part of a homogeneous transform T. The 3
% angles RPY=[R,P,Y] correspond to sequential rotations about the Z, Y and
% X axes respectively. Roll and yaw angles in [-pi,pi) while pitch angle
% in [-pi/2,pi/2).
%
% RPY = R2RPY(R) as above but the input is an orthonormal
% rotation matrix R (3x3).
%
% Notes::
% - This assume the 'zyx' axes
% - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
%   set to zero and Y is the sum (R+Y).
% - Translation component is ignored.
%
% See also  rpy2tr, tr2eul.
*/
TooN::Vector<3> r2rpy( const TooN::Matrix<3,3>& R );
TooN::Vector<3> tr2rpy( const TooN::Matrix<4,4>& T );

/*!
%RPY2TR/RPY2R Roll-pitch-yaw angles to homogeneous transform
%
% T = RPY2TR(ROLL, PITCH, YAW) is an SE(3) homogeneous
% transformation matrix (4x4) with zero translation and rotation equivalent
% to the specified roll, pitch, yaw angles angles. These correspond to
% rotations about the Z, Y, X axes respectively.
%
% T = RPY2TR(RPY) as above but the roll, pitch, yaw angles are
% taken from the vector (1x3) RPY=[ROLL,PITCH,YAW].
%
% Note::
% - AXES ZYX
%
% See also TR2RPY, RPY2R, EUL2TR.
*/
TooN::Matrix<3,3> rpy2r( double roll, double pitch, double yaw );
TooN::Matrix<3,3> rpy2r( TooN::Vector<3> &rpy );
TooN::Matrix<4,4> rpy2tr( double roll, double pitch, double yaw );
TooN::Matrix<4,4> rpy2tr( TooN::Vector<3> &rpy );

/*!
%TR2EUL/R2EUL Convert homogeneous transform to Euler angles
%
% EUL = TR2EUL(T) are the ZYZ Euler angles (1x3) corresponding to
% the rotational part of a homogeneous transform T (4x4). The 3 angles
% EUL=[PHI,THETA,PSI] correspond to sequential rotations about the Z, Y and
% Z axes respectively.
%
% EUL = TR2EUL(R) as above but the input is an orthonormal
% rotation matrix R (3x3).
%
% Notes::
% - AXES ZYZ
% - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
%   set to zero and PSI is the sum (PHI+PSI).
% - Translation component is ignored.
%
% See also  EUL2TR, TR2RPY.
*/
TooN::Vector<3> r2eul( const TooN::Matrix<3,3>& R );
TooN::Vector<3> tr2eul( const TooN::Matrix<4,4>& T );

/*!
%EUL2TR/EUL2R Convert Euler angles to homogeneous transform
%
% T = EUL2TR(PHI, THETA, PSI) is an SE(3) homogeneous
% transformation matrix (4x4) with zero translation and rotation equivalent
% to the specified Euler angles. These correspond to rotations about the Z,
% Y, Z axes respectively.
%
% R = EUL2R(EUL) as above but the Euler angles are taken from the
% vector (1x3)  EUL = [PHI THETA PSI].
%
% Options::
%  'deg'      Angles given in degrees (radians default)
%
% Note::
% - The vectors PHI, THETA, PSI must be of the same length.
% - The translational part is zero.
%
% See also EUL2R, RPY2TR, TR2EUL, SE3.eul.
*/
TooN::Matrix<3,3> eul2r( double phi, double theta, double psi );
TooN::Matrix<3,3> eul2r( const TooN::Vector<3> &eul );
TooN::Matrix<4,4> eul2tr( double phi, double theta, double psi );
TooN::Matrix<4,4> eul2tr( const TooN::Vector<3> &eul );

#endif