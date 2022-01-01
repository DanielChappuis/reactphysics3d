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

#ifndef REACTPHYSICS3D_DEBUG_RENDERER_H
#define REACTPHYSICS3D_DEBUG_RENDERER_H

// Libraries
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/engine/EventListener.h>
#include <string>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Forward declarations
class ConcaveMeshShape;
class ConvexMeshShape;
class HeightFieldShape;
class Collider;
class PhysicsWorld;

// Class DebugRenderer
/**
 * This class is used to display physics debug information directly into the user application view.
 * For instance, it is possible to display AABBs of colliders, colliders or contact points. This class
 * can be used to get the debug information as arrays of basic primitives (points, linges, triangles, ...).
 * You can use this to render physics debug information in your simulation on top of your object. Note that
 * you should use this only for debugging purpose and you should disable it when you compile the final release
 * version of your application because computing/rendering phyiscs debug information can be expensive.
 */
class DebugRenderer : public EventListener {

    public:

		/// Enumeration with basic colors
		enum class DebugColor {

            RED = 0xff0000,
            GREEN = 0x00ff00,
            BLUE = 0x0000ff,
            BLACK = 0x000000,
            WHITE = 0xffffff,
            YELLOW = 0xffff00,
            MAGENTA = 0xff00ff,
            CYAN = 0x00ffff,
		};

		/// Enumeration with debug item to renderer
		enum class DebugItem {

            /// Display the AABB for each collider
			COLLIDER_AABB				= 1 << 0,

            /// Display the fat AABB of the broad phase collision detection for each collider
			COLLIDER_BROADPHASE_AABB	= 1 << 1,

            /// Display the collision shape of each collider
			COLLISION_SHAPE				= 1 << 2,

            /// Display the contact points
			CONTACT_POINT				= 1 << 3,

            /// Display the contact normals
            CONTACT_NORMAL				= 1 << 4,
        };

		/// Struture that represents a line of the DebugRenderer
		struct DebugLine {
			
			/// Constructor
            DebugLine(const Vector3& point1, const Vector3& point2, uint32 color)
                :point1(point1), color1(color), point2(point2), color2(color) {

			}

            /// First point of the line
			Vector3 point1;

            /// Color of the first point
            uint32 color1;

            /// Second point of the line
            Vector3 point2;

            /// Color of the second point
            uint32 color2;
		};

		/// Struture that represents a triangle of the DebugRenderer
		struct DebugTriangle {
			
			/// Constructor
			DebugTriangle(const Vector3& point1, const Vector3& point2, const Vector3& point3, uint32 color)
                :point1(point1), color1(color), point2(point2), color2(color), point3(point3), color3(color) {

			}

            /// First point of the triangle
            Vector3 point1;

            /// Color of the first point
            uint32 color1;

            /// Second point of the triangle
            Vector3 point2;

            /// Color of the second point
            uint32 color2;

            /// Third point of the triangle
            Vector3 point3;

            /// Color of the third point
            uint32 color3;
		};

    private:

		// -------------------- Constants -------------------- //

		/// Number of sectors used to draw a sphere or a capsule
        static constexpr int NB_SECTORS_SPHERE = 18;

		/// Number of stacks used to draw a sphere or a capsule
        static constexpr int NB_STACKS_SPHERE = 10;

        /// Default radius of the sphere displayed to represent contact points
        static constexpr decimal DEFAULT_CONTACT_POINT_SPHERE_RADIUS = decimal(0.1);

        /// Default radius of the sphere displayed to represent contact points
        static constexpr decimal DEFAULT_CONTACT_NORMAL_LENGTH = decimal(1.0);

		// -------------------- Attributes -------------------- //

		/// Memory allocator
		MemoryAllocator& mAllocator;

        /// Array with all the debug lines
		Array<DebugLine> mLines;

        /// Array with all the debug triangles
		Array<DebugTriangle> mTriangles;

        /// 32-bits integer that contains all the flags of debug items to display
		uint32 mDisplayedDebugItems;

		/// Map a debug item with the color used to display it
		Map<DebugItem, uint32> mMapDebugItemWithColor;

        /// Radius of the sphere displayed to represent contact points
        decimal mContactPointSphereRadius;

        /// Lenght of contact normal
        decimal mContactNormalLength;

        // -------------------- Methods -------------------- //

		/// Draw an AABB
		void drawAABB(const AABB& aabb, uint32 color);

		/// Draw a box
		void drawBox(const Transform& transform, const Vector3& extents, uint32 color);

		/// Draw a sphere
		void drawSphere(const Vector3& position, decimal radius, uint32 color);

		/// Draw a capsule
		void drawCapsule(const Transform& transform, decimal radius, decimal height, uint32 color);

		/// Draw a convex mesh
		void drawConvexMesh(const Transform& transform, const ConvexMeshShape* convexMesh, uint32 color);

		/// Draw a concave mesh shape
		void drawConcaveMeshShape(const Transform& transform, const ConcaveMeshShape* concaveMeshShape, uint32 color);

		/// Draw a height field shape
		void drawHeightFieldShape(const Transform& transform, const HeightFieldShape* heightFieldShape, uint32 color);

		/// Draw the collision shape of a collider
		void drawCollisionShapeOfCollider(const Collider* collider, uint32 color);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        DebugRenderer(MemoryAllocator& allocator);

        /// Destructor
        ~DebugRenderer();

		/// Return the number of lines
		uint32 getNbLines() const;

        /// Return a reference to the array of lines
		const Array<DebugLine>& getLines() const;

		/// Return a pointer to the array of lines
		const DebugLine* getLinesArray() const;

		/// Return the number of triangles
		uint32 getNbTriangles() const;

        /// Return a reference to the array of triangles
		const Array<DebugTriangle>& getTriangles() const;

		/// Return a pointer to the array of triangles
		const DebugTriangle* getTrianglesArray() const;

		/// Return whether a debug item is displayed or not
		bool getIsDebugItemDisplayed(DebugItem item) const;

		/// Set whether a debug info is displayed or not
		void setIsDebugItemDisplayed(DebugItem item, bool isDisplayed);

        /// Get the contact point sphere radius
        decimal getContactPointSphereRadius() const;

        /// Set the contact point sphere radius
        void setContactPointSphereRadius(decimal radius);

        /// Return the length of contact normal
        decimal getContactNormalLength() const;

        /// Return the length of contact normal
        void setContactNormalLength(decimal contactNormalLength);

        /// Generate the rendering primitives (triangles, lines, ...) of a physics world
		void computeDebugRenderingPrimitives(const PhysicsWorld& world);

        /// Clear all the debugging primitives (points, lines, triangles, ...)
        void reset();

        /// Called when some contacts occur
        virtual void onContact(const CollisionCallback::CallbackData& callbackData) override;
};

// Return the number of lines
/**
 * @return The number of lines in the array of lines to draw
 */
RP3D_FORCE_INLINE uint32 DebugRenderer::getNbLines() const {
    return static_cast<uint32>(mLines.size());
}

// Return a reference to the array of lines
/**
 * @return The array of lines to draw
 */
RP3D_FORCE_INLINE const Array<DebugRenderer::DebugLine>& DebugRenderer::getLines() const {
	return mLines;
}

// Return a pointer to the array of lines
/**
 * @return A pointer to the first element of the lines array to draw
 */
RP3D_FORCE_INLINE const DebugRenderer::DebugLine* DebugRenderer::getLinesArray() const {
	return &(mLines[0]);
}

// Return the number of triangles
/**
 * @return The number of triangles in the array of triangles to draw
 */
RP3D_FORCE_INLINE uint32 DebugRenderer::getNbTriangles() const {
    return static_cast<uint32>(mTriangles.size());
}

// Return a reference to the array of triangles
/**
 * @return The array of triangles to draw
 */
RP3D_FORCE_INLINE const Array<DebugRenderer::DebugTriangle>& DebugRenderer::getTriangles() const {
	return mTriangles;
}

// Return a pointer to the array of triangles
/**
 * @return A pointer to the first element of the triangles array to draw
 */
RP3D_FORCE_INLINE const DebugRenderer::DebugTriangle* DebugRenderer::getTrianglesArray() const {
	return &(mTriangles[0]);
}

// Return whether a debug item is displayed or not
/**
 * @param item A debug item
 * @return True if the given debug item is being displayed and false otherwise
 */
RP3D_FORCE_INLINE bool DebugRenderer::getIsDebugItemDisplayed(DebugItem item) const {
	return mDisplayedDebugItems & static_cast<uint32>(item);
}

// Set whether a debug info is displayed or not
/**
 * @param item A debug item to draw
 * @param isDisplayed True if the given debug item has to be displayed and false otherwise
 */
RP3D_FORCE_INLINE void DebugRenderer::setIsDebugItemDisplayed(DebugItem item, bool isDisplayed) {
	const uint32 itemFlag = static_cast<uint32>(item);
	uint32 resetBit = ~(itemFlag);
	mDisplayedDebugItems &= resetBit;
	if (isDisplayed) {
		mDisplayedDebugItems |= itemFlag;
	}
}

// Get the contact point sphere radius
/**
 * @return The radius of the sphere used to display a contact point
 */
RP3D_FORCE_INLINE decimal DebugRenderer::getContactPointSphereRadius() const {
    return mContactPointSphereRadius;
}

// Set the contact point sphere radius
/**
 * @param radius The radius of the sphere used to display a contact point
 */
RP3D_FORCE_INLINE void DebugRenderer::setContactPointSphereRadius(decimal radius) {
    assert(radius > decimal(0.0));
    mContactPointSphereRadius = radius;
}


// Return the length of contact normal
/**
 * @return The length of the contact normal to display
 */
RP3D_FORCE_INLINE decimal DebugRenderer::getContactNormalLength() const {
    return mContactNormalLength;
}

// Return the length of contact normal
/**
 * @param contactNormalLength The length of the contact normal to display
 */
RP3D_FORCE_INLINE void DebugRenderer::setContactNormalLength(decimal contactNormalLength) {
    mContactNormalLength = contactNormalLength;
}

}

// Hash function for a DebugItem
namespace std {

  template <> struct hash<reactphysics3d::DebugRenderer::DebugItem> {

    size_t operator()(const reactphysics3d::DebugRenderer::DebugItem& debugItem) const {

        return std::hash<reactphysics3d::uint32>{}(static_cast<unsigned int>(debugItem));
    }
  };
}

#endif
