/*

    C++ implementation of Quaternion class from the Matlab Robotic Toolbox by Peter I. Corke.

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

#ifndef QUATERNION_H
#define QUATERNION_H

#include "PortingFunctions.h"

class Quaternion{

    private:

    protected:

        /*
            Scalar Part
        */
        double _s;

        /*
            Vector Part
        */
        TooN::Vector<3> _v;

    public:

    /*==========CONSTRUCTORS=========*/


    /*
        Full Constructor
    */
    Quaternion( double s, const TooN::Vector<3>& v );

    /*
        Zero Quaternion
    */
    Quaternion();

    /*
        Quaternion from a Vector<4>
    */
    Quaternion( const TooN::Vector<4>& q_vec );

    /*==========END ONSTRUCTORS=========*/


    /*==========Setters=========*/

    /*
        Set scalar part
    */
    virtual void setS( double s );

    /*
        Set vector part
    */
    virtual void setV( const TooN::Vector<3>& v );

    /*==========END Setters=========*/

    /*==========Getters=========*/

    /*
        Get scalar part
    */
    double getS() const;

    /*
        Get vector part
    */
    const TooN::Vector<3>& getV() const;

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
    void display() const;

    /*=========END VARIE=============*/

    /*======QUATERNION FUNCTIONS======*/

    /*
        Quaternion.conj Conjugate of a quaternion
        
         QI = Q.conj() is a quaternion object representing the conjugate of Q.
       
         Notes::
         - Conjugatation changes the sign of the vector component.
        
         See also Quaternion.inv.
    */
    Quaternion conj() const;


    /*
        Quaternion.inv Invert a quaternion
        
         QI = Q.inv() is a quaternion object representing the inverse of Q.
        
         See also Quaternion.conj.
    */
    Quaternion inv() const;

    /*
        Quaternion.unit Unitize a quaternion
        
         QU = Q.unit() is a UnitQuaternion object representing the same orientation as Q.
        
         Notes::
         See also Quaternion.norm, UnitQuaternion.
    */
    /* CANNOT BE HERE!
    UnitQuaternion unit() const;
    */

    /*
        Quaternion.norm Quaternion magnitude
        
         QN = Q.norm(Q) is the scalar norm or magnitude of the quaternion Q.
        
         Notes::
         - This is the Euclidean norm of the quaternion written as a 4-vector.
         - A unit-quaternion has a norm of one.
        
         See also Quaternion.inner, Quaternion.unit.
    */
    double norm() const;

    /*
        Quaternion.norm2 Quaternion square magnitude
        
         QN = Q.norm2(Q) is the square of the scalar norm or magnitude of the quaternion Q.
        
         See also Quaternion.inner, Quaternion.unit.
    */
    double norm2() const;

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

        Matlab Version (rememeber that matlab is 1-based):
            m = [q.s    -q.v(1) -q.v(2) -q.v(3)
                 q.v(1)  q.s    -q.v(3)  q.v(2)
                 q.v(2)  q.v(3)  q.s    -q.v(1)
                 q.v(3) -q.v(2)  q.v(1)  q.s];
    */
    TooN::Matrix<4,4> matrix() const;

    /*
        Quaternion.inner Quaternion inner product
        
         V = Q1.inner(Q2) is the inner (dot) product of two vectors (1x4),
         comprising the elements of Q1 and Q2 respectively.
        
         Notes::
         - Q1.inner(Q1) is the same as Q1.norm().
        
         See also Quaternion.norm.
    */
    double inner(const Quaternion& q2) const;



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
        Quaternion mtimes(const Quaternion& q2) const;
        Quaternion mtimes(double s) const;
        Quaternion operator*(const Quaternion& q2) const;
        Quaternion operator*(const double& s) const;


        /*
            Quaternion.mrdivide Quaternion quotient.
            
             Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
             Q/S     is the element-wise division of quaternion elements by the scalar S.
            
             Notes::
             - Overloaded operator '/'
            
             See also Quaternion.mtimes, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
        */
        Quaternion mrdivide(const Quaternion& q2) const;
        Quaternion mrdivide(double s) const;
        Quaternion operator/(const Quaternion& q2) const;
        Quaternion operator/(const double& s) const;

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
        Quaternion mpower(int p) const;
        Quaternion operator^(const int& p) const;

        /*
            PLUS Add quaternions
            
             Q1+Q2 is a Quaternion formed from the element-wise sum of quaternion elements.
            
             Notes::
             - Overloaded operator '+'
             - This is not a group operator, but it is useful to have the result as a
               quaternion.
            
             See also Quaternion.minus.
        */
        Quaternion plus( const Quaternion& q2 ) const;
        Quaternion operator+(const Quaternion& q2) const;


        /*
            Quaternion.minus Subtract quaternions
            
             Q1-Q2 is a Quaternion formed from the element-wise difference of quaternion elements.
            
             Notes::
             - Overloaded operator '-'
             - This is not a group operator, but it is useful to have the result as a
               quaternion.
            
             See also Quaternion.plus.
        */
        Quaternion minus( const Quaternion& q2 ) const;
        Quaternion operator-() const;
        Quaternion operator-(const Quaternion& q2) const;



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
        bool isequal(const Quaternion& q2) const;

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
        bool eq(const Quaternion& q2) const;
        bool operator==(const Quaternion& q2) const;

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
        bool ne( const Quaternion& q2 ) const;
        bool operator!=(const Quaternion& q2) const;

/*====== END RELATIONAL OPERATORS======*/

/*======TYPE CONVERSION METHODS======*/

    
        /*
            compact string representation of the quaternion's value
            as a 4-tuple.  If Q is a vector then S has one line per element.
        */
        //operator std::string() const;
        friend std::ostream &operator<<( std::ostream &output, const Quaternion &q );

        /*
            Quaternion.getDouble Convert a quaternion to a 4-element vector
            
             V = Q.double() is a row vector (1x4) comprising the quaternion elements,
             scalar then vector.
            
             elements [s vx vy vz].
        */
        TooN::Vector<4> getDouble() const;

};//END CLASS


/*============OPERATORS==========*/

//*CONJ*//
Quaternion conj(const Quaternion& q);
//*INV*//
Quaternion inv(const Quaternion& q );

//*UNIT*//
/* CANNOT BE HERE!
UnitQuaternion unit(const Quaternion& q){
    return q.unit();
}
*/

//*NORM*//
double norm(const Quaternion& q);

//*NORM2*//
double norm2(const Quaternion& q);

//*MATRIX*//
TooN::Matrix<4,4> matrix(const Quaternion& q);

//*INNER*//
double inner(const Quaternion& q1, const Quaternion& q2);

//*MTIMES*//
Quaternion mtimes(const Quaternion& q1, const Quaternion& q2);
Quaternion mtimes(const Quaternion& q1, double s);
Quaternion mtimes(double s, const Quaternion& q2);
//Quaternion operator*(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator*(const Quaternion& q1, const double& s);
Quaternion operator*(const double& s, const Quaternion& q2);

//*MRDIVIDE*//
Quaternion mrdivide(const Quaternion& q1, const Quaternion& q2);
Quaternion mrdivide(const Quaternion& q1, double s);
//Quaternion operator/(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator/(const Quaternion& q1, const double& s);

//*MPOWER*//
Quaternion mpower(const Quaternion& q1, int p);
//Quaternion operator^(const Quaternion& q1, const int& p);

//*PLUS*//
Quaternion plus( const Quaternion& q1, const Quaternion& q2 );
Quaternion plus( const Quaternion& q1, const TooN::Vector<4>& q2_vec );
Quaternion plus( const TooN::Vector<4>& q1_vec, const Quaternion& q2 );
//Quaternion operator+(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator+(const Quaternion& q1, const TooN::Vector<4>& q2_vec);
//Quaternion operator+(const TooN::Vector<4>& q1_vec, const Quaternion& q2 );

        
//*MINUS*//
Quaternion minus( const Quaternion& q );
Quaternion minus( const Quaternion& q1, const Quaternion& q2 );
Quaternion minus( const Quaternion& q1, const TooN::Vector<4>& q2_vec );
Quaternion minus( const TooN::Vector<4>& q1_vec, const Quaternion& q2 );
//Quaternion operator-(const Quaternion& q );
//Quaternion operator-(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator-(const Quaternion& q1, const TooN::Vector<4>& q2_vec);
//Quaternion operator-(const TooN::Vector<4>& q1_vec, const Quaternion& q2 );

//*EQ*//
bool isequal(const Quaternion& q1, const Quaternion& q2);
bool eq(const Quaternion& q1, const Quaternion& q2);
//bool operator==(const Quaternion& q1, const Quaternion& q2);

//*NE*//
bool ne(const Quaternion& q1, const Quaternion& q2);
//bool operator!=(const Quaternion& q1, const Quaternion& q2);

//*GETDOUBLE*//
TooN::Vector<4> getDouble(const Quaternion& q);


#endif








