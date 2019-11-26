/*

    AngVec Class

    This class represents a rotation in Axis–angle representation

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

#ifndef ANGVEC_H
#define ANGVEC_H

#include "TooN/TooN.h"


//!  Angle Vector class to represent a rotation. 
class AngVec{
    
    private:

    protected:

        //! Rotation Angle.
        double _theta;

        //! Rotation Axis.
        TooN::Vector<3> _vec;

    public:
        
        //! null rotation about the x-axis.
        AngVec();

        //! Rotation of angle theta about the vec axis
        /*!
            Note: axis will be normalized by the constructor
        */
        AngVec(double theta, const TooN::Vector<3>& vec);

        //! Rotation of angle theta about the vec axis
        /*!
            Note: axis will be normalized by the constructor
        */
        AngVec(const TooN::Vector<3>& vec, double theta);

    //==========GETTERS=========//
    
    //! get the angle
    double getAng() const;

    //! get the angle
    double getTheta() const;

    //! get the axis
    TooN::Vector<3> getVec() const;

    //! get the axis
    TooN::Vector<3> getAx() const;
    //==========================//

    //==========SETTERS=========//

    //! set the angle
    void setAng(double ang);

    //! set the angle
    void setTheta(double ang);

    //! set the axis
    /*!
        Note: axis will be normalized
    */
    void setVec( const TooN::Vector<3>& vec );

    //! set the axis
    /*!
        Note: axis will be normalized
    */
    void setAx( const TooN::Vector<3>& vec );

    //! set the angle and axis
    /*!
        Note: axis will be normalized
    */
    void set(double ang, const TooN::Vector<3>& vec );

    //! set the angle and axis
    /*!
        Note: axis will be normalized
    */
    void set(const TooN::Vector<3>& vec, double ang );
    //==========================//

};//END CLASS

#endif