/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

// Libraries
#include <stdexcept>

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Exception class for the mathematics library
    -------------------------------------------------------------------
*/

// Class MathematicsException
class MathematicsException : public std::runtime_error {
    public:
        MathematicsException(const std::string& msg="MathException");           // Constructor
        virtual ~MathematicsException() throw();                                // Destructor
        virtual const char* what() const throw();                               // Overriding the base exception method
};

// Class DivisionByZeroException
class DivisionByZeroException : public MathematicsException {
    public:
        DivisionByZeroException(const std::string& msg="DivisionByZeroException : Division by zero !");     // Constructor
        virtual ~DivisionByZeroException() throw();                                                         // Destructor
        virtual const char* what() const throw();                                                           // Overriding the base exception method
};

} // End of the ReactPhysics3D namespace

#endif
