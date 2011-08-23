/****************************************************************************
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

#ifndef SHAPE_H
#define SHAPE_H

// Libraries
#include <cassert>
#include "../mathematics/Vector3.h"
#include "../mathematics/Matrix3x3.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class Body;

/*  -------------------------------------------------------------------
    Class Shape :
        This abstract class represents the shape of a body that is used during
        the narrow-phase collision detection.
    -------------------------------------------------------------------
*/
class Shape {
    protected :
        Body* bodyPointer;              // Pointer to the owner body (not the abstract class Body but its derivative which is instanciable)

    public :
        Shape();                        // Constructor
        virtual ~Shape();               // Destructor

        Body* getBodyPointer() const;                                                                   // Return the body pointer
        void setBodyPointer(Body* bodyPointer);                                                         // Set the body pointer
        virtual Vector3 getLocalSupportPoint(const Vector3& direction, double margin=0.0) const=0;      // Return a local support point in a given direction
        virtual Vector3 getLocalExtents(double margin=0.0) const=0;                                     // Return the local extents in x,y and z direction
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, double mass) const=0;                 // Return the local inertia tensor of the shape
};

// Return the body pointer
inline Body* Shape::getBodyPointer() const {
    assert(bodyPointer != NULL);
    return bodyPointer;
}

// Set the body pointer
inline void Shape::setBodyPointer(Body* bodyPointer) {
    this->bodyPointer = bodyPointer;
}

}

#endif