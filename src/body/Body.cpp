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
#include "Body.h"
#include "BroadBoundingVolume.h"
#include "NarrowBoundingVolume.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Body::Body(const Transform& transform, double mass) throw(std::invalid_argument)
     : transform(transform), mass(mass), narrowBoundingVolume(0), aabb(0) {
    // Check if the mass is not larger than zero
    if (mass <= 0.0) {
        // We throw an exception
        throw std::invalid_argument("Exception in Body constructor : the mass has to be different larger than zero");
    }

    // Initialize the old transform
    oldTransform = transform;

    // Create the AABB for Broad-Phase collision detection
    aabb = new AABB(this);
}

// Destructor
Body::~Body() {
    /* TODO : DELETE THIS
    if (broadBoundingVolume) {
        delete broadBoundingVolume;
    }
    */
    
    if (narrowBoundingVolume) {
        delete narrowBoundingVolume;
    }

    // Delete the AABB
    delete aabb;
}

/* TODO : DELETE THIS
// Set the broad-phase bounding volume
void Body::setBroadBoundingVolume(BroadBoundingVolume* broadBoundingVolume) {
    assert(broadBoundingVolume);
    this->broadBoundingVolume = broadBoundingVolume;
    broadBoundingVolume->setBodyPointer(this);
}
*/

// Set the narrow-phase bounding volume
void Body::setNarrowBoundingVolume(NarrowBoundingVolume* narrowBoundingVolume) {
    assert(narrowBoundingVolume);
    this->narrowBoundingVolume = narrowBoundingVolume;
    narrowBoundingVolume->setBodyPointer(this);
}
