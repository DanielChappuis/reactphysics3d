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

#ifndef REACTPHYSICS3D_CONCAVE_VS_CONVEX_ALGORITHM_H
#define	REACTPHYSICS3D_CONCAVE_VS_CONVEX_ALGORITHM_H

// Libraries
#include "NarrowPhaseAlgorithm.h"
#include "collision/shapes/ConvexShape.h"
#include "collision/shapes/ConcaveShape.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Class ConvexVsTriangleCallback
/**
 * This class is used to encapsulate a callback method for
 * collision detection between the triangle of a concave mesh shape
 * and a convex shape.
 */
class ConvexVsTriangleCallback : public TriangleCallback {

    protected:

        /// Pointer to the collision detection object
        CollisionDetection* mCollisionDetection;

        /// Narrow-phase collision callback
        NarrowPhaseCallback* mNarrowPhaseCallback;

        /// Convex collision shape to test collision with
        const ConvexShape* mConvexShape;

        /// Proxy shape of the convex collision shape
        ProxyShape* mConvexProxyShape;

        /// Proxy shape of the concave collision shape
        ProxyShape* mConcaveProxyShape;

        /// Broadphase overlapping pair
        OverlappingPair* mOverlappingPair;

    public:

        /// Set the collision detection pointer
        void setCollisionDetection(CollisionDetection* collisionDetection) {
            mCollisionDetection = collisionDetection;
        }

        /// Set the narrow-phase collision callback
        void setNarrowPhaseCallback(NarrowPhaseCallback* callback) {
            mNarrowPhaseCallback = callback;
        }

        /// Set the convex collision shape to test collision with
        void setConvexShape(const ConvexShape* convexShape) {
            mConvexShape = convexShape;
        }

        /// Set the broadphase overlapping pair
        void setOverlappingPair(OverlappingPair* overlappingPair) {
            mOverlappingPair = overlappingPair;
        }

        /// Set the proxy shapes of the two collision shapes
        void setProxyShapes(ProxyShape* convexProxyShape, ProxyShape* concaveProxyShape) {
            mConvexProxyShape = convexProxyShape;
            mConcaveProxyShape = concaveProxyShape;
        }

        /// Test collision between a triangle and the convex mesh shape
        virtual void testTriangle(const Vector3* trianglePoints);

};

// Class ConcaveVsConvexAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between a concave collision shape and a convex collision shape. The idea is
 * to use the GJK collision detection algorithm to compute the collision between
 * the convex shape and each of the triangles in the concave shape.
 */
class ConcaveVsConvexAlgorithm : public NarrowPhaseAlgorithm {

    protected :

        // -------------------- Attributes -------------------- //

        /// Convex vs Triangle callback
        ConvexVsTriangleCallback mConvexVsTriangleCallback;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveVsConvexAlgorithm(const ConcaveVsConvexAlgorithm& algorithm);

        /// Private assignment operator
        ConcaveVsConvexAlgorithm& operator=(const ConcaveVsConvexAlgorithm& algorithm);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveVsConvexAlgorithm();

        /// Destructor
        virtual ~ConcaveVsConvexAlgorithm();

        /// Compute a contact info if the two bounding volume collide
        virtual void testCollision(const CollisionShapeInfo& shape1Info,
                                   const CollisionShapeInfo& shape2Info,
                                   NarrowPhaseCallback* narrowPhaseCallback);
};

}

#endif

