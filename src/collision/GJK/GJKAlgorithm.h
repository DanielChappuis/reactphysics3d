/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
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

#ifndef GJK_ALGORITHM_H
#define GJK_ALGORITHM_H

// Libraries
#include "../NarrowPhaseAlgorithm.h"
#include "../ContactInfo.h"
#include "../../body/NarrowBoundingVolume.h"
#include "../EPA/EPAAlgorithm.h"


// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const double REL_ERROR = 1.0e-3;
const double REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;

 // TODO : Improve the description here
/*  -------------------------------------------------------------------
    Class GJKAlgorithm :
        This class implements a narrow-phase collision detection algorithm. This
        algorithm uses the ISA-GJK algorithm and the EPA algorithm. This
        implementation is based on the implementation discussed in the book
        "Collision Detection in 3D Environments".
        This method implements the Hybrid Technique for calculating the
        penetration depth. The two objects are enlarged with a small margin. If
        the object intersection, the penetration depth is quickly computed using
        GJK algorithm on the original objects (without margin). If the
        original objects (without margin) intersect, we run again the GJK
        algorithm on the enlarged objects (with margin) to compute simplex
        polytope that contains the origin and give it to the EPA (Expanding
        Polytope Algorithm) to compute the correct penetration depth between the
        enlarged objects. 
    -------------------------------------------------------------------
*/
class GJKAlgorithm : public NarrowPhaseAlgorithm {
    private :
        EPAAlgorithm algoEPA;             // EPA Algorithm

        bool computePenetrationDepthForEnlargedObjects(const NarrowBoundingVolume* const boundingVolume1,
                                                       const NarrowBoundingVolume* const boundingVolume2,
                                                       ContactInfo*& contactInfo, Vector3D& v);             // Compute the penetration depth for enlarged objects

    public :
        GJKAlgorithm();           // Constructor
        ~GJKAlgorithm();          // Destructor

        virtual bool testCollision(const NarrowBoundingVolume* const boundingVolume1,
                                   const NarrowBoundingVolume* const boundingVolume2,
                                   ContactInfo*& contactInfo);                          // Return true and compute a contact info if the two bounding volume collide
};

} // End of the ReactPhysics3D namespace

// TODO : Check what would be a correct value for the OBJECT_MARGIN constant

#endif
