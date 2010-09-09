/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
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
