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