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
#include "ContactInfo.h"

using namespace reactphysics3d;

// Constructor for SAT
ContactInfo::ContactInfo(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth)
            : body1(body1), body2(body2), normal(normal), penetrationDepth(penetrationDepth), point1(0.0, 0.0, 0.0), point2(0.0, 0.0, 0.0) {

}

// Constructor for GJK
ContactInfo::ContactInfo(Body* const body1, Body* const body2, const Vector3D& normal,
            double penetrationDepth, const Vector3D& point1, const Vector3D& point2)
            : body1(body1), body2(body2), normal(normal), penetrationDepth(penetrationDepth), point1(point1), point2(point2) {
    
}
