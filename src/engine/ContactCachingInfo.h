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

#ifndef CONTACT_CACHING_INFO_H
#define	CONTACT_CACHING_INFO_H

// Libraries
#include "../shapes/BoxShape.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Structure ContactCachingInfo :
       This structure contains informations used to cache the last lambda
       value of each contact constraint. The last lambda value is used
       as the init lambda value in the constraint solver in the next
       time step to improve the convergence rate of the constraint solver.
    -------------------------------------------------------------------
*/
struct ContactCachingInfo {
    public:
        Body* body1;                            // Body pointer of the first bounding volume
        Body* body2;                            // Body pointer of the second bounding volume
        std::vector<Vector3D> positions;        // Positions of the contact points
        std::vector<double> lambdas;            // Last lambdas value for the constraint

        ContactCachingInfo(Body* body1, Body* body2, const std::vector<Vector3D>& positions, const std::vector<double>& lambdas);   // Constructor
};

} // End of the ReactPhysics3D namespace

#endif

