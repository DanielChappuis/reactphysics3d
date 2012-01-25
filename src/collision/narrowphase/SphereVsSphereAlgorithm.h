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

#ifndef SPHERE_VS_SPHERE_ALGORITHM_H
#define	SPHERE_VS_SPHERE_ALGORITHM_H

// Libraries
#include "../../body/Body.h"
#include "../ContactInfo.h"
#include "NarrowPhaseAlgorithm.h"


// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class SphereVsSphereAlgorithm :
        This class is used to compute the narrow-phase collision detection
        between two sphere colliders.
    -------------------------------------------------------------------
*/
class SphereVsSphereAlgorithm : public NarrowPhaseAlgorithm {
    protected :
        
    public :
        SphereVsSphereAlgorithm(CollisionDetection& collisionDetection);   // Constructor
        virtual ~SphereVsSphereAlgorithm();                                // Destructor

        virtual bool testCollision(const Collider* collider1, const Transform& transform1,
                                   const Collider* collider2, const Transform& transform2,
                                   ContactInfo*& contactInfo);  // Return true and compute a contact info if the two bounding volume collide
};

} // End of reactphysics3d namespace

#endif

