/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

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
