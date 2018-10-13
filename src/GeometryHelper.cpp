/*

    Geometry Helper Lib

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

*/

#include "GeometryHelper.h"

using namespace TooN;
using namespace std;


double norm(const Matrix<3> &M)
{
    SVD<> Msvd(M);
    Vector<> sigma=Msvd.get_diagonal();
    return sigma[0];
}


Vector<> abs(const Vector<> &v)
{
    Vector<> v_out = Zeros(v.size());
    for( int i = 0; i<v.size(); i++ )
        v_out[i] = abs(v[i]);
    return v_out;
}


double sum(const Vector<> &v)
{
    double out = 0.0;
    for( int i = 0; i<v.size(); i++ )
        out += v[i];
    return out;
}


Matrix<3,3> rotx( double angle ){
    double c = cos(angle);
    double s = sin(angle);
    return Data(
                1.0, 0.0, 0.0,
                0.0,   c,  -s,
                0.0,   s,   c
    );
}

Matrix<3,3> roty( double angle ){
    double c = cos(angle);
    double s = sin(angle);
    return Data(
                c,   0.0,   s,
                0.0, 1.0, 0.0,
                -s,  0.0,   c
    );
}

Matrix<3,3> rotz( double angle ){
    double c = cos(angle);
    double s = sin(angle);
    return Data(
                  c,  -s, 0.0,
                  s,   c, 0.0,
                0.0, 0.0, 1.0
    );
}


double sign( double n, int zero_out ){
    if(n == 0.0)
        return (double)zero_out;
    else
        return (double)((n > 0) - (n < 0));
}

double sign( double n ){
    return (double)((n > 0) - (n < 0));
}






int index_max( const Vector<>& v ){
    if( v.size() < 1 )
        return -1;

    int index = 0;
    for( int i = 1; i < v.size(); i++ )
        if( v[i] > v[index] )
            index = i;
    return index;
}