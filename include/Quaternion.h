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

/*! \file Quaternion.h
    \brief C++ implementation of Quaternion class from the Matlab Robotic Toolbox by Peter I. Corke.
*/

#include "PortingFunctions.h"

//!  Quaternion class to represent a Quaternion.
/*!
    Note that a Quaternion does not necessary represent a rotation. See the UnitQuaternion class.
    See Quaternion.h for more functions.
    \sa UnitQuaternion, Quaternion.h
*/ 
class Quaternion{

    private:

    protected:

        //!  Scalar Part.
        double _s;

        //!  Vector Part
        TooN::Vector<3> _v;

    public:

    /*==========CONSTRUCTORS=========*/

    //!  Full Constructor
    /*!
        Costructor that takes the scalar part s and the vectorial part v
        \param s scalar part
        \param v vectorial part
    */
    Quaternion( double s, const TooN::Vector<3>& v );

    //!  Zero Quaternion
    /*!
        The quaternion 0,<<0,0,0>>
    */
    Quaternion();

    //! Quaternion from a Vector<4>
    /*!
        Quaternion from a vector.
        \param q_vec is the vector [w x y z] where w is the scalar part
    */
    Quaternion( const TooN::Vector<4>& q_vec );

    /*==========END ONSTRUCTORS=========*/


    /*==========Setters=========*/

    //! Set scalar part
    /*!
        \param s scalar part
    */
    virtual void setS( double s );

    //! Set vector part
    /*!
        \param v vector part
    */
    virtual void setV( const TooN::Vector<3>& v );

    /*==========END Setters=========*/

    /*==========Getters=========*/

    //! Get scalar part
    double getS() const;

    //! Get vector part
    const TooN::Vector<3>& getV() const;

    /*==========END Setters=========*/

    /*=========VARIE==============*/

    //! Display Quaternion on the std output
    /*!
        
         Q.display() displays a compact string representation of the quaternion's value
         as a 4-tuple.
        
         Notes::
         - The vector part is displayed with double brackets << 1, 0, 0 >> to
           distinguish it from a UnitQuaternion which displays as < 1, 0, 0 >
        
    */
    void display() const;

    /*=========END VARIE=============*/

    /*======QUATERNION FUNCTIONS======*/

    //! Conjugate of a quaternion
    /*!        
         QI = Q.conj() is a quaternion object representing the conjugate of Q.
       
         Notes::
         - Conjugatation changes the sign of the vector component.
        
         \sa inv
    */
    Quaternion conj() const;


    //! Invert a quaternion
    /*!        
         QI = Q.inv() is a quaternion object representing the inverse of Q.
        
         \sa conj
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

    /*!
        Quaternion.norm Quaternion magnitude
        
         QN = Q.norm(Q) is the scalar norm or magnitude of the quaternion Q.
        
         Notes::
         - This is the Euclidean norm of the quaternion written as a 4-vector.
         - A unit-quaternion has a norm of one.
        
         See also Quaternion.inner, Quaternion.unit.
    */
    double norm() const;

    /*!
        Quaternion.norm2 Quaternion square magnitude
        
         QN = Q.norm2(Q) is the square of the scalar norm or magnitude of the quaternion Q.
        
         See also Quaternion.inner, Quaternion.unit.
    */
    double norm2() const;

    /*!
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

    /*!
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
        
        //! Multiply a quaternion object
        /*!
            Quaternion.mtimes Multiply a quaternion object
            
             Q1*Q2   is a quaternion formed by the Hamilton product of two quaternions.
             Q*S     is the element-wise multiplication of quaternion elements by the scalar S.
             S*Q     is the element-wise multiplication of quaternion elements by the scalar S.
            
             Notes::
             - Overloaded operator '*'
            
             See also Quaternion.mrdivide, Quaternion.mpower.
        */
        Quaternion mtimes(const Quaternion& q2) const;

        //! Multiply a quaternion object
        /*!
            \sa mtimes
        */
        Quaternion mtimes(double s) const;

        //! Multiply a quaternion object
        /*!
            \sa mtimes
        */
        Quaternion operator*(const Quaternion& q2) const;

        //! Multiply a quaternion object
        /*!
            \sa mtimes
        */
        Quaternion operator*(const double& s) const;


        //! Quaternion quotient.
        /*!
            Quaternion.mrdivide Quaternion quotient.
            
             Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
             Q/S     is the element-wise division of quaternion elements by the scalar S.
            
             Notes::
             - Overloaded operator '/'
            
             See also Quaternion.mtimes, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
        */
        Quaternion mrdivide(const Quaternion& q2) const;

        //! Quaternion quotient.
        /*!
            \sa mrdivide
        */
        Quaternion mrdivide(double s) const;

        //! Quaternion quotient.
        /*!
            \sa mrdivide
        */
        Quaternion operator/(const Quaternion& q2) const;

        //! Quaternion quotient.
        /*!
            \sa mrdivide
        */
        Quaternion operator/(const double& s) const;

        //! Raise quaternion to integer power
        /*!
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

        //! Raise quaternion to integer power
        /*!
            \sa mpower
        */
        Quaternion operator^(const int& p) const;

        //! Add quaternions
        /*!
            PLUS Add quaternions
            
             Q1+Q2 is a Quaternion formed from the element-wise sum of quaternion elements.
            
             Notes::
             - Overloaded operator '+'
             - This is not a group operator, but it is useful to have the result as a
               quaternion.
            
             See also Quaternion.minus.
        */
        Quaternion plus( const Quaternion& q2 ) const;

        //! Add quaternions
        /*!
            \sa plus
        */
        Quaternion operator+(const Quaternion& q2) const;


        //! Subtract quaternions
        /*!
            Quaternion.minus Subtract quaternions
            
             Q1-Q2 is a Quaternion formed from the element-wise difference of quaternion elements.
            
             Notes::
             - Overloaded operator '-'
             - This is not a group operator, but it is useful to have the result as a
               quaternion.
            
             See also Quaternion.plus.
        */
        Quaternion minus( const Quaternion& q2 ) const;

        //! Minus
        /*!
            This is -Q
            
             -Q is a Quaternion formed by -w,<<-x,-y,-z>>.
            
             \sa minus
        */
        Quaternion operator-() const;

        //! Subtract quaternions
        /*!
            \sa minus
        */
        Quaternion operator-(const Quaternion& q2) const;



/*======END ARITHMETIC OPERATORS======*/

/*======RELATIONAL OPERATORS======*/


        //! Test quaternion element equality
        /*!   
             Q1.ISEQUAL(Q2) is true if the quaternions Q1 and Q2 are equal.
            
             Notes::
             - Used by test suite verifyEqual in addition to eq().
             - Invokes eq().
            
             See also Quaternion.eq.
        */
        bool isequal(const Quaternion& q2) const;

        //! Test quaternion element equality
        /*!   
             Q1==Q2 is true if the quaternions Q1 and Q2 are equal.
            
             Notes::
             - Overloaded operator '=='.
             - This method is invoked for unit Quaternions where Q and -Q represent
               the equivalent rotation, so non-equality does not mean rotations are not
               equivalent.
            
             See also Quaternion.ne.
        */
        bool eq(const Quaternion& q2) const;

        //! Test quaternion element equality
        /*!   
             \sa eq
        */
        bool operator==(const Quaternion& q2) const;

        //! Test quaternion inequality
        /*!            
             Q1 != Q2 is true if the quaternions Q1 and Q2 are not equal.
            
             Notes::
             - Overloaded operator '!='
             - Note that for unit Quaternions Q and -Q are the equivalent
               rotation, so non-equality does not mean rotations are not
               equivalent.
            
             See also Quaternion.eq.
        */
        bool ne( const Quaternion& q2 ) const;

        //! Test quaternion inequality
        /*!            
             \sa ne
        */
        bool operator!=(const Quaternion& q2) const;

/*====== END RELATIONAL OPERATORS======*/

/*======TYPE CONVERSION METHODS======*/

    
        //! ostream operator
        /*!
            compact string representation of the quaternion's value
            as a 4-tuple.
        */
        friend std::ostream &operator<<( std::ostream &output, const Quaternion &q );
        //operator std::string() const;

        //! Convert a quaternion to a 4-element vector
        /*!            
             V = Q.double() is a Vector<4> comprising the quaternion elements,
             scalar then vector.
            
             [s vx vy vz].
        */
        TooN::Vector<4> getDouble() const;

};//END CLASS


/*============OPERATORS==========*/

//*CONJ*//
//! Conjugate of q
Quaternion conj(const Quaternion& q);
//*INV*//
//! Inverse of q
Quaternion inv(const Quaternion& q );

//*UNIT*//
/* CANNOT BE HERE!
UnitQuaternion unit(const Quaternion& q){
    return q.unit();
}
*/

//*NORM*//
//! norm of q
double norm(const Quaternion& q);

//*NORM2*//
//! square norm of q
double norm2(const Quaternion& q);

//*MATRIX*//
//! Matrix representation
/*!
    \sa Quaternion::matrix()
*/
TooN::Matrix<4,4> matrix(const Quaternion& q);

//*INNER*//
//! inner product
double inner(const Quaternion& q1, const Quaternion& q2);

//*MTIMES*//

//! Multiply a quaternion object
Quaternion mtimes(const Quaternion& q1, const Quaternion& q2);

//! Multiply a quaternion object
Quaternion mtimes(const Quaternion& q1, double s);

//! Multiply a quaternion object
Quaternion mtimes(double s, const Quaternion& q2);
//Quaternion operator*(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator*(const Quaternion& q1, const double& s);

//! Multiply a quaternion object
Quaternion operator*(const double& s, const Quaternion& q2);

//*MRDIVIDE*//

//! Quaternion quotient
Quaternion mrdivide(const Quaternion& q1, const Quaternion& q2);

//! Quaternion quotient
Quaternion mrdivide(const Quaternion& q1, double s);
//Quaternion operator/(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator/(const Quaternion& q1, const double& s);

//*MPOWER*//

//! Raise quaternion to integer power
Quaternion mpower(const Quaternion& q1, int p);
//Quaternion operator^(const Quaternion& q1, const int& p);

//*PLUS*//

//!Add quaternions
Quaternion plus( const Quaternion& q1, const Quaternion& q2 );

//!Add quaternions
Quaternion plus( const Quaternion& q1, const TooN::Vector<4>& q2_vec );

//!Add quaternions
Quaternion plus( const TooN::Vector<4>& q1_vec, const Quaternion& q2 );

//Quaternion operator+(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator+(const Quaternion& q1, const TooN::Vector<4>& q2_vec);
//Quaternion operator+(const TooN::Vector<4>& q1_vec, const Quaternion& q2 );

        
//*MINUS*//

//! Subtract quaternions
Quaternion minus( const Quaternion& q );

//! Subtract quaternions
Quaternion minus( const Quaternion& q1, const Quaternion& q2 );

//! Subtract quaternions
Quaternion minus( const Quaternion& q1, const TooN::Vector<4>& q2_vec );

//! Subtract quaternions
Quaternion minus( const TooN::Vector<4>& q1_vec, const Quaternion& q2 );

//Quaternion operator-(const Quaternion& q );
//Quaternion operator-(const Quaternion& q1, const Quaternion& q2);
//Quaternion operator-(const Quaternion& q1, const TooN::Vector<4>& q2_vec);
//Quaternion operator-(const TooN::Vector<4>& q1_vec, const Quaternion& q2 );

//*EQ*//

//! Test quaternion element equality
bool isequal(const Quaternion& q1, const Quaternion& q2);

//! Test quaternion element equality
bool eq(const Quaternion& q1, const Quaternion& q2);

//bool operator==(const Quaternion& q1, const Quaternion& q2);

//*NE*//

//! Test quaternion inequality
bool ne(const Quaternion& q1, const Quaternion& q2);
//bool operator!=(const Quaternion& q1, const Quaternion& q2);

//*GETDOUBLE*//

//! Convert a quaternion to a 4-element vector
/*!            
    V = Q.double() is a Vector<4> comprising the quaternion elements,
    scalar then vector.
            
    [s vx vy vz].
*/
TooN::Vector<4> getDouble(const Quaternion& q);


#endif








