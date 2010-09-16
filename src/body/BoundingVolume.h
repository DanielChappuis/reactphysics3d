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

#ifndef BOUNDING_VOLUME_H
#define BOUNDING_VOLUME_H

// Libraries
#include "Body.h"
#include "../mathematics/mathematics.h"
#include <cassert>


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class BoundingVolume :
        This class represents the volume that contains a rigid body.
        This volume will be used to compute the collisions with others
        bodies.
    -------------------------------------------------------------------
*/
class BoundingVolume {
    protected :
        Body* body;                         // Pointer to the body

    public :
        BoundingVolume();                   // Constructor
        virtual ~BoundingVolume();          // Destructor

        Body* getBodyPointer() const;       // Return the body pointer
        void setBodyPointer(Body* body);    // Set the body pointer

        virtual void update(const Vector3D& newCenter, const Quaternion& rotationQuaternion)=0;      // Update the orientation of the bounding volume according to the new orientation of the body
        #ifdef VISUAL_DEBUG
            virtual void draw() const=0;                                                             // Display the bounding volume (only for testing purpose)
        #endif
};

// Return the body pointer
inline Body* BoundingVolume::getBodyPointer() const {
    assert(body != NULL);
    return body;
}

// Set the body pointer
inline void BoundingVolume::setBodyPointer(Body* bodyPointer) {
    this->body = bodyPointer;
}


} // End of the ReactPhysics3D namespace

#endif
