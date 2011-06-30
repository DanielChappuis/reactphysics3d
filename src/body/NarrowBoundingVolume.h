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

#ifndef NARROW_BOUNDING_VOLUME_H
#define NARROW_BOUNDING_VOLUME_H

// Libraries
#include "BoundingVolume.h"
#include "AABB.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class NarrowBoundingVolume :
        This class represents the volume that contains a rigid body
        This volume will be used to compute the narrow-phase collision
        detection.
    -------------------------------------------------------------------
*/
class NarrowBoundingVolume : public BoundingVolume {
    protected :

    public :
        NarrowBoundingVolume();                 // Constructor
        virtual ~NarrowBoundingVolume();        // Destructor

        // TODO : DELETE virtual AABB* computeAABB() const=0;                                                        // Return the corresponding AABB
        virtual Vector3D getSupportPoint(const Vector3D& direction, double margin=0.0) const=0;     // Return a support point in a given direction
        virtual Vector3D getLocalExtents() const=0;                                                 // Return the local extents in x,y and z direction
};

}

#endif