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
#include "Vector3.h"
#include <iostream>
#include <cassert>
#include <vector>

// Namespaces
using namespace reactphysics3d;

// Constructor of the class Vector3D
Vector3::Vector3() {
    values[0] = 0.0;
    values[1] = 0.0;
    values[2] = 0.0;
}

// Constructor with arguments
Vector3::Vector3(double x, double y, double z) {
    values[0] = x;
    values[1] = y;
    values[2] = z;
}

// Copy-constructor
Vector3::Vector3(const Vector3& vector) {
    values[0] = vector.values[0];
    values[1] = vector.values[1];
    values[2] = vector.values[2];
}

// Destructor
Vector3::~Vector3() {

}

// Return the corresponding unit vector
Vector3 Vector3::getUnit() const {
    double lengthVector = length();

    assert(lengthVector != 0.0);

    // Compute and return the unit vector
    double lengthInv = 1.0 / lengthVector;
    return Vector3(values[0] * lengthInv, values[1] * lengthInv, values[2] * lengthInv);
}

// Return two unit orthogonal vectors of the current vector
Vector3 Vector3::getOneOrthogonalVector() const {
    assert(!this->isZero());

    // Compute a first orthogonal vector
    Vector3 vector1;
    if (!approxEqual(values[0], 0.0)) {   // If x != 0
        vector1.setY(values[0]);
        vector1.setZ((-2*values[0]*values[1]*values[2] + 2*values[0]*values[2])/(2*(values[2]*values[2] + values[0]*values[0])));
        vector1.setX((-values[0]*values[1]-values[2]*vector1.getZ())/values[0]);
    }
    else if (!approxEqual(values[1], 0.0)) { // If y != 0
        vector1.setZ(values[1]);
        vector1.setX((-2*values[0]*values[1]*values[2] + 2*values[0]*values[1])/(2*(values[1]*values[1] + values[0]*values[0])));
        vector1.setY((-values[2]*values[1]-values[0]*vector1.getX())/values[1]);
    }
    else if (!approxEqual(values[2], 0.0)) { // If z != 0
        vector1.setX(values[2]);
        vector1.setY((-2*values[0]*values[1]*values[2] + 2*values[1]*values[2])/(2*(values[2]*values[2] + values[1]*values[1])));
        vector1.setZ((-values[0]*values[2]-values[1]*vector1.getY())/values[2]);
    }

    assert(vector1.isUnit());
    return vector1;
}
