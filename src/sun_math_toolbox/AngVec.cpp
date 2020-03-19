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

#include "sun_math_toolbox/AngVec.h"

using namespace TooN;
using namespace std;
using namespace sun;

//==========CONSTRUCTORS=========//
AngVec::AngVec()
{
  _theta = 0.0;
  _vec = makeVector(1.0, 0.0, 0.0);
}

AngVec::AngVec(double theta, const Vector<3>& vec)
{
  _theta = theta;
  _vec = unit(vec);
}

AngVec::AngVec(const Vector<3>& vec, double theta) : AngVec(theta, vec)
{
}

//==========END CONSTRUCTORS=========//

//==========GETTERS=========//
double AngVec::getAng() const
{
  return _theta;
}
double AngVec::getTheta() const
{
  return getAng();
}
Vector<3> AngVec::getVec() const
{
  return _vec;
}
Vector<3> AngVec::getAx() const
{
  return getVec();
}
//=========END GETTERS========//

//==========SETTERS=========//
void AngVec::setAng(double ang)
{
  _theta = ang;
}
void AngVec::setTheta(double ang)
{
  setAng(ang);
}
void AngVec::setVec(const Vector<3>& vec)
{
  _vec = unit(vec);
}
void AngVec::setAx(const TooN::Vector<3>& vec)
{
  setVec(vec);
}
void AngVec::set(double ang, const Vector<3>& vec)
{
  setAng(ang);
  setVec(vec);
}
void AngVec::set(const Vector<3>& vec, double ang)
{
  set(ang, vec);
}
//========END SETTERS=========//