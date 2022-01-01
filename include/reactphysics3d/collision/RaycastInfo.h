/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_RAYCAST_INFO_H
#define REACTPHYSICS3D_RAYCAST_INFO_H

// Libraries
#include <reactphysics3d/mathematics/Vector3.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;
class Collider;
class CollisionShape;
struct Ray;

// Structure RaycastInfo
/**
 * This structure contains the information about a raycast hit.
 */
struct RaycastInfo {

    private:

    public:

        // -------------------- Attributes -------------------- //

        /// Hit point in world-space coordinates
        Vector3 worldPoint;

        /// Surface normal at hit point in world-space coordinates
        Vector3 worldNormal;

        /// Fraction distance of the hit point between point1 and point2 of the ray
        /// The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
        decimal hitFraction;

        /// Mesh subpart index that has been hit (only used for triangles mesh and -1 otherwise)
        int meshSubpart;

        /// Hit triangle index (only used for triangles mesh and -1 otherwise)
        int triangleIndex;

        /// Pointer to the hit collision body
        CollisionBody* body;

        /// Pointer to the hit collider
        Collider* collider;

        // -------------------- Methods -------------------- //

        /// Constructor
        RaycastInfo() : meshSubpart(-1), triangleIndex(-1), body(nullptr), collider(nullptr) {

        }

        /// Destructor
        ~RaycastInfo() = default;

        /// Deleted copy constructor
        RaycastInfo(const RaycastInfo& raycastInfo) = delete;

        /// Deleted assignment operator
        RaycastInfo& operator=(const RaycastInfo& raycastInfo) = delete;
};

// Class RaycastCallback
/**
 * This class can be used to register a callback for ray casting queries.
 * You should implement your own class inherited from this one and implement
 * the notifyRaycastHit() method. This method will be called for each collider
 * that is hit by the ray.
 */
class RaycastCallback {

    public:

        // -------------------- Methods -------------------- //

        /// Destructor
        virtual ~RaycastCallback() {

        }

        /// This method will be called for each collider that is hit by the
        /// ray. You cannot make any assumptions about the order of the
        /// calls. You should use the return value to control the continuation
        /// of the ray. The returned value is the next maxFraction value to use.
        /// If you return a fraction of 0.0, it means that the raycast should
        /// terminate. If you return a fraction of 1.0, it indicates that the
        /// ray is not clipped and the ray cast should continue as if no hit
        /// occurred. If you return the fraction in the parameter (hitFraction
        /// value in the RaycastInfo object), the current ray will be clipped
        /// to this fraction in the next queries. If you return -1.0, it will
        /// ignore this collider and continue the ray cast.
        /**
         * @param raycastInfo Information about the raycast hit
         * @return Value that controls the continuation of the ray after a hit
         */
        virtual decimal notifyRaycastHit(const RaycastInfo& raycastInfo)=0;

};

/// Structure RaycastTest
struct RaycastTest {

    public:

        /// User callback class
        RaycastCallback* userCallback;

        /// Constructor
        RaycastTest(RaycastCallback* callback) {
            userCallback = callback;

        }

        /// Ray cast test against a collider
        decimal raycastAgainstShape(Collider* shape, const Ray& ray);
};

}

#endif
