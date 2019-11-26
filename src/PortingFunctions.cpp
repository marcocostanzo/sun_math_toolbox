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

#include "PortingFunctions.h"

using namespace TooN;
using namespace std;

////////////////////////////////////////////////////
Matrix<3,3> skew( const Vector<3> &v ){
	return Data(  0.0, -v[2],  v[1],
				 v[2],   0.0, -v[0],
				-v[1],  v[0],   0.0  );
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
bool isRot(const Matrix<> &m){
    if( m.num_cols() == m.num_rows() && m.num_cols() == 3 )
        if( fabs((determinant(m) - 1.0)) < 10.0*GEOMETRY_HELPER_EPSILON  )
            return true;
    return false;
}

bool isHomog(const Matrix<> &m){
    if( m.num_cols() == m.num_rows() && m.num_cols() == 4 )
        if( m[3][0] == 0.0 && m[3][1] == 0.0 && m[3][2] == 0.0 && m[3][3] == 1.0 )
            return isRot( t2r(m) );
    return false;
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
Matrix<4,4> transl( double x, double y, double z ){
    return Data(
                    1.0, 0.0, 0.0,   x,
                    0.0, 1.0, 0.0,   y,
                    0.0, 0.0, 1.0,   z,
                    0.0, 0.0, 0.0, 1.0
    );
}
Matrix<4,4> transl( const Vector<3>& tr ){
    return transl( tr[0], tr[1], tr[2] );
}
Vector<3> transl( const Matrix<4,4>& T ){
    return makeVector( T[0][3], T[1][3], T[2][3] );
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
Matrix<3,3> t2r(const Matrix<4,4> &T){
    return T.slice<0,0,3,3>();
}

Matrix<4,4> r2t(const Matrix<3,3> &R){
    return Data(
                    R[0][0], R[0][1], R[0][2], 0.0,
                    R[1][0], R[1][1], R[1][2], 0.0,
                    R[2][0], R[2][1], R[2][2], 0.0,
                        0.0,     0.0,     0.0, 1.0
    );
}

Matrix<4,4> rt2tr(const Matrix<3,3> &R, const Vector<3> &t){
    Matrix<4,4> T = transl(t);
    T.slice<0,0,3,3>() = R;
    return T;
}
Matrix<4,4> rt2tr(const Vector<3> &t, const Matrix<3,3> &R){
    return rt2tr( R , t );
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
Vector<3> r2rpy( const Matrix<3,3>& R ){

        //Check matrix
        if( !isRot(R) ){
            throw std::invalid_argument("[PORTINGFUNCIONS] Error in r2rpy( const Matrix<3,3>& R ): R is not a rotation Matrix");
        }

        //old ZYX order (as per Paul book)

        Vector<3> rpy = Zeros;

        if( fabs( fabs(R[2][0]) - 1.0 ) < GEOMETRY_HELPER_EPSILON  ){ //when |R31| == 1
            //singularity

            rpy[0] = 0;  // roll is zero
            if( R[2][0] < 0.0 ){
                rpy[2] = -atan2( R[0][1] , R[0][2] );  // R-Y
            } else {
                rpy[2] = atan2( -R[0][1], -R[0][2] );  // R+Y
            }
            rpy[1] = -asin( R[2][0] );
        } else {
            rpy[0] = atan2( R[2][1], R[2][2] );  // R
            rpy[2] = atan2( R[1][0], R[0][0] );  // Y
                
            rpy[1] = -atan( R[2][0] * cos( rpy[0] ) / R[2][2] );
        }

        return rpy;

}
Vector<3> tr2rpy( const Matrix<4,4>& T ){
    return r2rpy( t2r(T) );
}

Matrix<3,3> rpy2r( double roll, double pitch, double yaw ){
    return rotz(yaw) * roty(pitch) * rotx(roll);
}
Matrix<3,3> rpy2r( Vector<3> &rpy ){
    return rpy2r( rpy[0], rpy[1], rpy[2] );
}
Matrix<4,4> rpy2tr( double roll, double pitch, double yaw ){
    return r2t( rpy2r( roll, pitch, yaw ) );
}
Matrix<4,4> rpy2tr( Vector<3> &rpy ){
    return r2t( rpy2r( rpy ) );
}
////////////////////////////////////////////////////

////////////////////////////////////////////////////
Vector<3> r2eul( const Matrix<3,3>& R ){

    //Check matrix
    if( !isRot(R) ){
        throw std::invalid_argument("[PORTINGFUNCIONS] Error in r2eul( const Matrix<3,3>& R ): R is not a rotation Matrix");        
    }

    /* Method as per Paul, p 69.
       euler = [phi theta psi]
    */

    Vector<3> eul = Zeros;

    if( abs( R[0][2] ) < GEOMETRY_HELPER_EPSILON && abs( R[1][2] ) < GEOMETRY_HELPER_EPSILON ){
        //singularity
        eul[0] = 0.0;
        double sp = 0.0;
        double cp = 0.0;
        eul[1] = atan2( cp*R[0][2] + sp*R[1][2], R[2][2]);
        eul[2] = atan2( -sp * R[0][0] + cp * R[1][0], -sp*R[0][1] + cp*R[1][1] );
    } else {
        // non singular
        eul[0] = atan2( R[1][2] , R[0][2] );
        double sp = sin(eul[0]);
        double cp = cos(eul[0]);
        eul[1] = atan2( cp*R[0][2] + sp*R[1][2] , R[2][2] );
        eul[2] = atan2(-sp * R[0][0] + cp * R[1][0], -sp*R[0][1] + cp*R[1][1]);
    }

    return eul;

}
Vector<3> tr2eul( const Matrix<4,4>& T ){
    return r2eul( T.slice<0,0,3,3>() );
}

Matrix<3,3> eul2r( double phi, double theta, double psi ){
    return rotz(phi) * roty(theta) * rotz(psi);
}
Matrix<3,3> eul2r( const Vector<3> &eul ){
    return eul2r( eul[0], eul[1], eul[2] );
}
Matrix<4,4> eul2tr( double phi, double theta, double psi ){
    return r2t( eul2r( phi, theta, psi ) );
}
Matrix<4,4> eul2tr( const Vector<3> &eul ){
    return r2t( eul2r( eul ) );
}
////////////////////////////////////////////////////