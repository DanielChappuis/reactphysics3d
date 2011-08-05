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
#include "../shapes/Shape.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Body::Body(const Transform& transform, Shape* shape, double mass)
     : shape(shape), transform(transform), mass(mass) {
    assert(mass > 0.0);
    assert(shape);

    isMotionEnabled = true;
    isCollisionEnabled = true;
    interpolationFactor = 0.0;

    // Initialize the old transform
    oldTransform = transform;

    // Create the AABB for broad-phase collision detection
    aabb = new AABB(transform, shape->getLocalExtents(OBJECT_MARGIN));
}

// Destructor
Body::~Body() {

    // Delete the AABB
    delete aabb;
}
