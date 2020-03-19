/*

    AngVec Class

    This class represents a rotation in Axis–angle representation

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

#ifndef ANGVEC_H
#define ANGVEC_H

#include "TooN/TooN.h"

namespace sun
{
//!  Angle Vector class to represent a rotation.
class AngVec
{
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
      Note: axis will be normalized by the constructor.

      \param theta rotation angle
      \param vec rotation axis
  */
  AngVec(double theta, const TooN::Vector<3>& vec);

  //! Rotation of angle theta about the vec axis.
  /*!
      Note: axis will be normalized by the constructor.

      \param theta rotation angle.
      \param vec rotation axis.
  */
  AngVec(const TooN::Vector<3>& vec, double theta);

  //==========GETTERS=========//

  //! get the angle.
  /*!
      Is the same as getTheta()
      \return The rotation angle.
      \sa getTheta
  */
  double getAng() const;

  //! get the angle.
  /*!
      Is the same as getAng()
      \return The rotation angle.
      \sa getAng
  */
  double getTheta() const;

  //! get the axis.
  /*!
      Is the same as getAx()
      \return The rotation axis.
      \sa getAx
  */
  TooN::Vector<3> getVec() const;

  //! get the axis.
  /*!
      Is the same as getVec()
      \return The rotation axis.
      \sa getVec
  */
  TooN::Vector<3> getAx() const;
  //==========================//

  //==========SETTERS=========//

  //! set the angle
  /*!
      Is the same as setTheta
      \param ang The rotation angle.
      \sa setTheta, set
  */
  void setAng(double ang);

  //! set the angle
  /*!
      Is the same as setAng
      \param ang The rotation angle.
      \sa setAng, set
  */
  void setTheta(double ang);

  //! set the axis
  /*!
      Is the same as setAx
      Note: axis will be normalized
      \param vec the rotation axis
      \sa setAx, set
  */
  void setVec(const TooN::Vector<3>& vec);

  //! set the axis
  /*!
      Is the same as setVec
      Note: axis will be normalized
      \param vec the rotation axis
      \sa setVec, set
  */
  void setAx(const TooN::Vector<3>& vec);

  //! set the angle and axis
  /*!
      Note: axis will be normalized
      \param ang The rotation angle.
      \param vec the rotation axis.
      \sa setAng, setVec
  */
  void set(double ang, const TooN::Vector<3>& vec);

  //! set the angle and axis
  /*!
      Note: axis will be normalized
      \param ang The rotation angle.
      \param vec the rotation axis.
      \sa setAng, setVec
  */
  void set(const TooN::Vector<3>& vec, double ang);
  //==========================//

};  // END CLASS

}  // namespace sun

#endif