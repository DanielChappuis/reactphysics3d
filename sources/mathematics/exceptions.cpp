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

