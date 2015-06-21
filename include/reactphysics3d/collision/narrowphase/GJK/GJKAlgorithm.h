/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_GJK_ALGORITHM_H
#define REACTPHYSICS3D_GJK_ALGORITHM_H

// Libraries
#include "reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h"
#include "reactphysics3d/collision/narrowphase/EPA/EPAAlgorithm.h"
#include "reactphysics3d/collision/shapes/CollisionShape.h"
#include "reactphysics3d/constraint/ContactPoint.h"

/// ReactPhysics3D Namespace
namespace reactphysics3d {

// Constants
const decimal REL_ERROR = decimal(1.0e-3);
const decimal REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
const int MAX_ITERATIONS_GJK_RAYCAST = 32;

// Class GJKAlgorithm
/**
 * This class implements a narrow-phase collision detection algorithm. This
 * algorithm uses the ISA-GJK algorithm and the EPA algorithm. This
 * implementation is based on the implementation discussed in the book
 * "Collision Detection in Interactive 3D Environments" by Gino van den Bergen.
 * This method implements the Hybrid Technique for calculating the
 * penetration depth. The two objects are enlarged with a small margin. If
 * the object intersects in their margins, the penetration depth is quickly
 * computed using the GJK algorithm on the original objects (without margin).
 * If the original objects (without margin) intersect, we run again the GJK
 * algorithm on the enlarged objects (with margin) to compute simplex
 * polytope that contains the origin and give it to the EPA (Expanding
 * Polytope Algorithm) to compute the correct penetration depth between the
 * enlarged objects.
 */
class GJKAlgorithm : public NarrowPhaseAlgorithm {

    private :

        // -------------------- Attributes -------------------- //

        /// EPA Algorithm
        EPAAlgorithm mAlgoEPA;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        GJKAlgorithm(const GJKAlgorithm& algorithm);

        /// Private assignment operator
        GJKAlgorithm& operator=(const GJKAlgorithm& algorithm);

        /// Compute the penetration depth for enlarged objects.
        bool computePenetrationDepthForEnlargedObjects(ProxyShape* collisionShape1,
                                                       const Transform& transform1,
                                                       ProxyShape* collisionShape2,
                                                       const Transform& transform2,
                                                       ContactPointInfo*& contactInfo, Vector3& v);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        GJKAlgorithm(MemoryAllocator& memoryAllocator);

        /// Destructor
        ~GJKAlgorithm();

        /// Return true and compute a contact info if the two bounding volumes collide.
        virtual bool testCollision(ProxyShape* collisionShape1, ProxyShape* collisionShape2,
                                   ContactPointInfo*& contactInfo);

        /// Use the GJK Algorithm to find if a point is inside a convex collision shape
        bool testPointInside(const Vector3& localPoint, ProxyShape* collisionShape);

        /// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
        bool raycast(const Ray& ray, ProxyShape* collisionShape, RaycastInfo& raycastInfo);
};

}

#endif
