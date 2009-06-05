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

// Libraries
#include "exceptions.h"

// Namespaces
using namespace reactphysics3d;


// Constructor of the MathematicsException class
MathematicsException::MathematicsException(const std::string& msg)
              :std::runtime_error(msg) {

}

// Destructor of the MathException class
MathematicsException::~MathematicsException() throw() {

}

// Overriden exception base class method
const char* MathematicsException::what() const throw() {
    return std::runtime_error::what();
}

// Constructor of the DivisionByZeroException class
DivisionByZeroException::DivisionByZeroException(const std::string& msg)
                        :MathematicsException(msg) {

}

// Destructor of the DivisionByZeroException class
DivisionByZeroException::~DivisionByZeroException() throw() {

}

// Overriden exception base class method
const char* DivisionByZeroException::what() const throw() {
    return MathematicsException::what();
}

