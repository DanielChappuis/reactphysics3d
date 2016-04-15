/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <unordered_map>

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

        /// Concave collision shape
        const ConcaveShape* mConcaveShape;

        /// Proxy shape of the convex collision shape
        ProxyShape* mConvexProxyShape;

        /// Proxy shape of the concave collision shape
        ProxyShape* mConcaveProxyShape;

        /// Broadphase overlapping pair
        OverlappingPair* mOverlappingPair;

        /// Used to sort ContactPointInfos according to their penetration depth
        static bool contactsDepthCompare(const ContactPointInfo& contact1,
                                         const ContactPointInfo& contact2);

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

        /// Set the concave collision shape
        void setConcaveShape(const ConcaveShape* concaveShape) {
            mConcaveShape = concaveShape;
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

// Class SmoothMeshContactInfo
/**
 * This class is used to store data about a contact with a triangle for the smooth
 * mesh algorithm.
 */
class SmoothMeshContactInfo {

    public:

        ContactPointInfo contactInfo;
        bool isFirstShapeTriangle;
        Vector3 triangleVertices[3];

        /// Constructor
        SmoothMeshContactInfo(const ContactPointInfo& contact, bool firstShapeTriangle, const Vector3& trianglePoint1,
                              const Vector3& trianglePoint2, const Vector3& trianglePoint3)
            : contactInfo(contact) {
            isFirstShapeTriangle = firstShapeTriangle;
            triangleVertices[0] = trianglePoint1;
            triangleVertices[1] = trianglePoint2;
            triangleVertices[2] = trianglePoint3;
        }

};

struct ContactsDepthCompare {
    bool operator()(const SmoothMeshContactInfo& contact1, const SmoothMeshContactInfo& contact2)
    {
        return contact1.contactInfo.penetrationDepth < contact2.contactInfo.penetrationDepth;
    }
};

/// Method used to compare two smooth mesh contact info to sort them
//inline static bool contactsDepthCompare(const SmoothMeshContactInfo& contact1,
//                                        const SmoothMeshContactInfo& contact2) {
//    return contact1.contactInfo.penetrationDepth < contact2.contactInfo.penetrationDepth;
//}

// Class SmoothCollisionNarrowPhaseCallback
/**
 * This class is used as a narrow-phase callback to get narrow-phase contacts
 * of the concave triangle mesh to temporary store them in order to be used in
 * the smooth mesh collision algorithm if this one is enabled.
 */
class SmoothCollisionNarrowPhaseCallback : public NarrowPhaseCallback {

    private:

        std::vector<SmoothMeshContactInfo>& mContactPoints;


    public:

        // Constructor
        SmoothCollisionNarrowPhaseCallback(std::vector<SmoothMeshContactInfo>& contactPoints)
          : mContactPoints(contactPoints) {

        }


        /// Called by a narrow-phase collision algorithm when a new contact has been found
        virtual void notifyContact(OverlappingPair* overlappingPair,
                                   const ContactPointInfo& contactInfo);

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

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveVsConvexAlgorithm(const ConcaveVsConvexAlgorithm& algorithm);

        /// Private assignment operator
        ConcaveVsConvexAlgorithm& operator=(const ConcaveVsConvexAlgorithm& algorithm);

        /// Process the concave triangle mesh collision using the smooth mesh collision algorithm
        void processSmoothMeshCollision(OverlappingPair* overlappingPair,
                                        std::vector<SmoothMeshContactInfo> contactPoints,
                                        NarrowPhaseCallback* narrowPhaseCallback);

        /// Add a triangle vertex into the set of processed triangles
        void addProcessedVertex(std::unordered_multimap<int, Vector3>& processTriangleVertices,
                                const Vector3& vertex);

        /// Return true if the vertex is in the set of already processed vertices
        bool hasVertexBeenProcessed(const std::unordered_multimap<int, Vector3>& processTriangleVertices,
                                    const Vector3& vertex) const;

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

// Add a triangle vertex into the set of processed triangles
inline void ConcaveVsConvexAlgorithm::addProcessedVertex(std::unordered_multimap<int, Vector3>& processTriangleVertices, const Vector3& vertex) {
    processTriangleVertices.insert(std::make_pair(int(vertex.x * vertex.y * vertex.z), vertex));
}

}

#endif

