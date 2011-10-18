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

#ifndef NARROW_PHASE_ALGORITHM_H
#define NARROW_PHASE_ALGORITHM_H

// Libraries
#include "../../body/Body.h"
#include "../ContactInfo.h"
#include "../CollisionDetection.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class NarrowPhaseAlgorithm :
        This class is an abstract class that represents an algorithm
        used to perform the narrow phase of a collision detection. The
        goal of the narrow phase algorithm is to compute contact
        informations of a collision between two bodies.
    -------------------------------------------------------------------
*/
class NarrowPhaseAlgorithm {
    protected :
        CollisionDetection& collisionDetection;  // Reference to the collision detection object
        
    public :
        NarrowPhaseAlgorithm(CollisionDetection& collisionDetection);   // Constructor
        virtual ~NarrowPhaseAlgorithm();                                // Destructor

        virtual bool testCollision(const Shape* shape1, const Transform& transform1,
                                   const Shape* shape2, const Transform& transform2,
                                   ContactInfo*& contactInfo)=0;  // Return true and compute a contact info if the two bounding volume collide
};

} // End of reactphysics3d namespace

#endif


