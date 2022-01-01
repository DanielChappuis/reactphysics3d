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

#ifndef REACTPHYSICS3D_COLLISION_SHAPE_H
#define REACTPHYSICS3D_COLLISION_SHAPE_H

// Libraries
#include <cassert>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/containers/Array.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class Profiler;
class DefaultPoolAllocator;
class AABB;
class Transform;
struct Ray;
struct RaycastInfo;
struct Vector3;
class Matrix3x3;
    
/// Type of collision shapes
enum class CollisionShapeType {SPHERE, CAPSULE, CONVEX_POLYHEDRON, CONCAVE_SHAPE};
const int NB_COLLISION_SHAPE_TYPES = 4;

/// Names of collision shapes
enum class CollisionShapeName { TRIANGLE, SPHERE, CAPSULE, BOX, CONVEX_MESH, TRIANGLE_MESH, HEIGHTFIELD };

// Declarations
class Collider;
class CollisionBody;

// Class CollisionShape
/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class CollisionShape {
        
    protected :

        // -------------------- Attributes -------------------- //

        /// Type of the collision shape
        CollisionShapeType mType;

		/// Name of the colision shape
		CollisionShapeName mName;

        /// Unique identifier of the shape inside an overlapping pair
        uint32 mId;

        /// Array of the colliders associated with this shape
        Array<Collider*> mColliders;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif
        
        // -------------------- Methods -------------------- //

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, Collider* collider) const=0;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const=0;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const = 0;

        /// Assign a new collider to the collision shape
        void addCollider(Collider* collider);

        /// Remove an assigned collider from the collision shape
        void removeCollider(Collider* collider);

        /// Notify all the assign colliders that the size of the collision shape has changed
        void notifyColliderAboutChangedSize();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionShape(CollisionShapeName name, CollisionShapeType type, MemoryAllocator& allocator);

        /// Destructor
        virtual ~CollisionShape() = default;

        /// Deleted copy-constructor
        CollisionShape(const CollisionShape& shape) = delete;

        /// Deleted assignment operator
        CollisionShape& operator=(const CollisionShape& shape) = delete;

		/// Return the name of the collision shape
		CollisionShapeName getName() const;

        /// Return the type of the collision shape
        CollisionShapeType getType() const;

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const=0;

        /// Return true if the collision shape is a polyhedron
        virtual bool isPolyhedron() const=0;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const=0;

        /// Return the id of the shape
        uint32 getId() const;

        /// Return the local inertia tensor of the collision shapes
        virtual Vector3 getLocalInertiaTensor(decimal mass) const=0;

        /// Compute and return the volume of the collision shape
        virtual decimal getVolume() const=0;

        /// Compute the world-space AABB of the collision shape given a transform
        virtual void computeAABB(AABB& aabb, const Transform& transform) const;

        /// Return the string representation of the shape
        virtual std::string to_string() const=0;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
        virtual void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class Collider;
        friend class CollisionBody;
        friend class RigidBody;
        friend class PhysicsWorld;
        friend class BroadPhaseSystem;
};

// Return the name of the collision shape
/**
* @return The name of the collision shape (box, sphere, triangle, ...)
*/
RP3D_FORCE_INLINE CollisionShapeName CollisionShape::getName() const {
	return mName;
}

// Return the type of the collision shape
/**
 * @return The type of the collision shape (sphere, capsule, convex polyhedron, concave mesh)
 */
RP3D_FORCE_INLINE CollisionShapeType CollisionShape::getType() const {
    return mType;
}

// Return the id of the shape
RP3D_FORCE_INLINE uint32 CollisionShape::getId() const {
   return mId;
}

// Assign a new collider to the collision shape
RP3D_FORCE_INLINE void CollisionShape::addCollider(Collider* collider) {
    mColliders.add(collider);
}

// Remove an assigned collider from the collision shape
RP3D_FORCE_INLINE void CollisionShape::removeCollider(Collider* collider) {
    mColliders.remove(collider);
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void CollisionShape::setProfiler(Profiler* profiler) {

	mProfiler = profiler;
}

#endif

}

#endif
