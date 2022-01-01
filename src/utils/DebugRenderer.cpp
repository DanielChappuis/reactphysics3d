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

// Libraries
#include <reactphysics3d/utils/DebugRenderer.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <cassert>
#include <reactphysics3d/collision/shapes/ConvexMeshShape.h>
#include <reactphysics3d/collision/shapes/ConcaveMeshShape.h>
#include <reactphysics3d/collision/shapes/HeightFieldShape.h>
#include <reactphysics3d/collision/shapes/BoxShape.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/containers/Pair.h>

using namespace reactphysics3d;

// Constructor
DebugRenderer::DebugRenderer(MemoryAllocator& allocator)
              :mAllocator(allocator), mLines(allocator), mTriangles(allocator), mDisplayedDebugItems(0), mMapDebugItemWithColor(allocator),
               mContactPointSphereRadius(DEFAULT_CONTACT_POINT_SPHERE_RADIUS), mContactNormalLength(DEFAULT_CONTACT_NORMAL_LENGTH) {

    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLIDER_AABB, static_cast<uint32>(DebugColor::MAGENTA)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLIDER_BROADPHASE_AABB, static_cast<uint32>(DebugColor::YELLOW)));
	mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLISION_SHAPE, static_cast<uint32>(DebugColor::GREEN)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::CONTACT_POINT, static_cast<uint32>(DebugColor::RED)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::CONTACT_NORMAL, static_cast<uint32>(DebugColor::WHITE)));
}

// Destructor
DebugRenderer::~DebugRenderer() {

}

// Clear all the debugging primitives (points, lines, triangles, ...)
void DebugRenderer::reset() {

	mLines.clear();
	mTriangles.clear();
}

// Draw an AABB
void DebugRenderer::drawAABB(const AABB& aabb, uint32 color) {
	
	const Vector3& min = aabb.getMin();
	const Vector3& max = aabb.getMax();

	// Bottom edges
	mLines.add(DebugLine(Vector3(min.x, min.y, max.z), Vector3(max.x, min.y, max.z), color));
	mLines.add(DebugLine(Vector3(max.x, min.y, max.z),  Vector3(max.x, min.y, min.z), color));
	mLines.add(DebugLine(Vector3(max.x, min.y, min.z), Vector3(min.x, min.y, min.z), color));
	mLines.add(DebugLine(Vector3(min.x, min.y, min.z), Vector3(min.x, min.y, max.z), color));

	// Top edges
	mLines.add(DebugLine(Vector3(min.x, max.y, max.z), Vector3(max.x, max.y, max.z), color));
	mLines.add(DebugLine(Vector3(max.x, max.y, max.z), Vector3(max.x, max.y, min.z), color));
	mLines.add(DebugLine(Vector3(max.x, max.y, min.z), Vector3(min.x, max.y, min.z), color));
	mLines.add(DebugLine(Vector3(min.x, max.y, min.z), Vector3(min.x, max.y, max.z), color));

	// Side edges
	mLines.add(DebugLine(Vector3(min.x, min.y, max.z), Vector3(min.x, max.y, max.z), color));
	mLines.add(DebugLine(Vector3(max.x, min.y, max.z), Vector3(max.x, max.y, max.z), color));
	mLines.add(DebugLine(Vector3(max.x, min.y, min.z), Vector3(max.x, max.y, min.z), color));
	mLines.add(DebugLine(Vector3(min.x, min.y, min.z), Vector3(min.x, max.y, min.z), color));
}

// Draw a box
void DebugRenderer::drawBox(const Transform& transform, const Vector3& halfExtents, uint32 color) {

	Vector3 vertices[8];

	// Vertices
	vertices[0] = transform * Vector3(-halfExtents.x, -halfExtents.y, halfExtents.z);
	vertices[1] = transform * Vector3(halfExtents.x, -halfExtents.y, halfExtents.z);
	vertices[2] = transform * Vector3(halfExtents.x, -halfExtents.y, -halfExtents.z);
	vertices[3] = transform * Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z);
	vertices[4] = transform * Vector3(-halfExtents.x, halfExtents.y, halfExtents.z);
	vertices[5] = transform * Vector3(halfExtents.x, halfExtents.y, halfExtents.z);
	vertices[6] = transform * Vector3(halfExtents.x, halfExtents.y, -halfExtents.z);
	vertices[7] = transform * Vector3(-halfExtents.x, halfExtents.y, -halfExtents.z);

	// Triangle faces
	mTriangles.add(DebugTriangle(vertices[0], vertices[1], vertices[5], color));
	mTriangles.add(DebugTriangle(vertices[0], vertices[5], vertices[4], color));
	mTriangles.add(DebugTriangle(vertices[1], vertices[2], vertices[6], color));
	mTriangles.add(DebugTriangle(vertices[1], vertices[6], vertices[5], color));
	mTriangles.add(DebugTriangle(vertices[2], vertices[3], vertices[6], color));
	mTriangles.add(DebugTriangle(vertices[3], vertices[7], vertices[6], color));
	mTriangles.add(DebugTriangle(vertices[0], vertices[7], vertices[3], color));
	mTriangles.add(DebugTriangle(vertices[0], vertices[4], vertices[7], color));
	mTriangles.add(DebugTriangle(vertices[0], vertices[2], vertices[1], color));
	mTriangles.add(DebugTriangle(vertices[0], vertices[3], vertices[2], color));
	mTriangles.add(DebugTriangle(vertices[5], vertices[6], vertices[4], color));
	mTriangles.add(DebugTriangle(vertices[4], vertices[6], vertices[7], color));
}

/// Draw a sphere
void DebugRenderer::drawSphere(const Vector3& position, decimal radius, uint32 color) {

    Vector3 vertices[(NB_SECTORS_SPHERE + 1) * (NB_STACKS_SPHERE + 1) + (NB_SECTORS_SPHERE + 1)];
	
	// Vertices
    const decimal sectorStep = 2 * PI_RP3D / NB_SECTORS_SPHERE;
    const decimal stackStep = PI_RP3D / NB_STACKS_SPHERE;
	
    for (uint32 i = 0; i <= NB_STACKS_SPHERE; i++) {

        const decimal stackAngle = PI_RP3D / 2 - i * stackStep;
		const decimal radiusCosStackAngle = radius * std::cos(stackAngle);
		const decimal z = radius * std::sin(stackAngle);

        for (uint32 j = 0; j <= NB_SECTORS_SPHERE; j++) {
		
			const decimal sectorAngle = j * sectorStep;
			const decimal x = radiusCosStackAngle * std::cos(sectorAngle);
			const decimal y = radiusCosStackAngle * std::sin(sectorAngle);

            vertices[i * (NB_SECTORS_SPHERE + 1) + j] = position + Vector3(x, y, z);
		}
	}

	// Faces
    for (uint32 i = 0; i < NB_STACKS_SPHERE; i++) {

        uint32 a1 = i * (NB_SECTORS_SPHERE + 1);
        uint32 a2 = a1 + NB_SECTORS_SPHERE + 1;

        for (uint32 j = 0; j < NB_SECTORS_SPHERE; j++, a1++, a2++) {
		
			// 2 triangles per sector except for the first and last stacks

			if (i != 0) {
			
				mTriangles.add(DebugTriangle(vertices[a1], vertices[a2], vertices[a1 + 1], color));
			}

			if (i != (NB_STACKS_SPHERE - 1)) {
				
				mTriangles.add(DebugTriangle(vertices[a1 + 1], vertices[a2], vertices[a2 + 1], color));
			}
		}
	}
}

// Draw a capsule
void DebugRenderer::drawCapsule(const Transform& transform, decimal radius, decimal height, uint32 color) {

    Vector3 vertices[(NB_SECTORS_SPHERE + 1) * (NB_STACKS_SPHERE + 1) + (NB_SECTORS_SPHERE + 1)];

    const decimal halfHeight = decimal(0.5) * height;

	// Use an even number of stacks
    const uint32 nbStacks = NB_STACKS_SPHERE % 2 == 0 ? NB_STACKS_SPHERE : NB_STACKS_SPHERE - 1;
    const uint32 nbHalfStacks = nbStacks / 2;
	
	// Vertices
    const decimal sectorStep = 2 * PI_RP3D / NB_SECTORS_SPHERE;
    const decimal stackStep = PI_RP3D / nbStacks;
	
    uint32 vertexIndex = 0;
	
	// Top cap sphere vertices
    for (uint32 i = 0; i <= nbHalfStacks; i++) {

        const decimal stackAngle = PI_RP3D / 2 - i * stackStep;
		const decimal radiusCosStackAngle = radius * std::cos(stackAngle);
        const decimal y = radius * std::sin(stackAngle);

        for (uint32 j = 0; j <= NB_SECTORS_SPHERE; j++) {
		
			const decimal sectorAngle = j * sectorStep;
            const decimal x = radiusCosStackAngle * std::sin(sectorAngle);
            const decimal z = radiusCosStackAngle * std::cos(sectorAngle);

            assert(vertexIndex < (NB_SECTORS_SPHERE + 1) * (nbStacks + 1) + (NB_SECTORS_SPHERE + 1));
			vertices[vertexIndex] = transform * Vector3(x, y + halfHeight, z);

			vertexIndex++;
		}
	}

	// Bottom cap sphere vertices
    for (uint32 i = 0; i <= nbHalfStacks; i++) {

        const decimal stackAngle = PI_RP3D / 2 - (nbHalfStacks + i) * stackStep;
		const decimal radiusCosStackAngle = radius * std::cos(stackAngle);
        const decimal y = radius * std::sin(stackAngle);

        for (uint32 j = 0; j <= NB_SECTORS_SPHERE; j++) {
		
			const decimal sectorAngle = j * sectorStep;
            const decimal x = radiusCosStackAngle * std::sin(sectorAngle);
            const decimal z = radiusCosStackAngle * std::cos(sectorAngle);

            assert(vertexIndex < (NB_SECTORS_SPHERE + 1) * (nbStacks + 1) + (NB_SECTORS_SPHERE + 1));
            vertices[vertexIndex] = transform * Vector3(x, y - halfHeight, z);

			vertexIndex++;
		}
	}

	// Faces of the top cap sphere
    for (uint32 i = 0; i < nbHalfStacks; i++) {

        uint32 a1 = i * (NB_SECTORS_SPHERE + 1);
        uint32 a2 = a1 + NB_SECTORS_SPHERE + 1;

        for (uint32 j = 0; j < NB_SECTORS_SPHERE; j++, a1++, a2++) {
		
			// 2 triangles per sector except for the first stack

            if (i != 0) {

                mTriangles.add(DebugTriangle(vertices[a1], vertices[a2], vertices[a1 + 1], color));
            }

            mTriangles.add(DebugTriangle(vertices[a1 + 1], vertices[a2], vertices[a2 + 1], color));
        }
	}

	// Faces of the bottom cap sphere
    for (uint32 i = 0; i < nbHalfStacks; i++) {

        uint32 a1 = (nbHalfStacks + 1) * (NB_SECTORS_SPHERE + 1) + i * (NB_SECTORS_SPHERE + 1);
        uint32 a2 = a1 + NB_SECTORS_SPHERE + 1;

        for (uint32 j = 0; j < NB_SECTORS_SPHERE; j++, a1++, a2++) {
		
			// 2 triangles per sector except for the last stack

            mTriangles.add(DebugTriangle(vertices[a1], vertices[a2], vertices[a1 + 1], color));

            if (i != (nbHalfStacks - 1)) {

                mTriangles.add(DebugTriangle(vertices[a1 + 1], vertices[a2], vertices[a2 + 1], color));
            }
        }
	}

	// Faces of the cylinder between the two spheres
    uint32 a1 = nbHalfStacks * (NB_SECTORS_SPHERE + 1);
    uint32 a2 = a1 + NB_SECTORS_SPHERE + 1;
    for (uint32 i = 0; i < NB_SECTORS_SPHERE; i++, a1++, a2++) {

		mTriangles.add(DebugTriangle(vertices[a1 + 1], vertices[a2], vertices[a2 + 1], color));
		mTriangles.add(DebugTriangle(vertices[a1], vertices[a2], vertices[a1 + 1], color));
	}
}

// Draw a convex mesh
void DebugRenderer::drawConvexMesh(const Transform& transform, const ConvexMeshShape* convexMesh, uint32 color) {

	// For each face of the convex mesh
	for (uint32 f = 0; f < convexMesh->getNbFaces(); f++) {

		const HalfEdgeStructure::Face& face = convexMesh->getFace(f);
		assert(face.faceVertices.size() >= 3);

		// Perform a fan triangulation of the convex polygon face
        const uint32 nbFaceVertices = static_cast<uint32>(face.faceVertices.size());
        for (uint32 v = 2; v < nbFaceVertices; v++) {

            uint32 v1Index = face.faceVertices[v - 2];
            uint32 v2Index = face.faceVertices[v - 1];
            uint32 v3Index = face.faceVertices[v];

            Vector3 v1 = convexMesh->getVertexPosition(v1Index);
            Vector3 v2 = convexMesh->getVertexPosition(v2Index);
            Vector3 v3 = convexMesh->getVertexPosition(v3Index);

            v1 = transform * v1;
            v2 = transform * v2;
            v3 = transform * v3;

            mTriangles.add(DebugTriangle(v1, v2, v3, color));
		}
	}
}

// Draw a concave mesh shape
void DebugRenderer::drawConcaveMeshShape(const Transform& transform, const ConcaveMeshShape* concaveMeshShape, uint32 color) {

	// For each sub-part of the mesh
    for (uint32 p = 0; p < concaveMeshShape->getNbSubparts(); p++) {

		// For each triangle of the sub-part
        for (uint32 t = 0; t < concaveMeshShape->getNbTriangles(p); t++) {
			
			Vector3 triangleVertices[3];
			concaveMeshShape->getTriangleVertices(p, t, triangleVertices);

            triangleVertices[0] = transform * triangleVertices[0];
            triangleVertices[1] = transform * triangleVertices[1];
            triangleVertices[2] = transform * triangleVertices[2];

            mTriangles.add(DebugTriangle(triangleVertices[0], triangleVertices[1], triangleVertices[2], color));
		}
	}
}

// Draw a height field shape
void DebugRenderer::drawHeightFieldShape(const Transform& transform, const HeightFieldShape* heightFieldShape, uint32 color) {

    // For each sub-grid points (except the last ones one each dimension)
    for (int i = 0; i < heightFieldShape->getNbColumns() - 1; i++) {
        for (int j = 0; j < heightFieldShape->getNbRows() - 1; j++) {

            // Compute the four point of the current quad
            Vector3 p1 = heightFieldShape->getVertexAt(i, j);
            Vector3 p2 = heightFieldShape->getVertexAt(i, j + 1);
            Vector3 p3 = heightFieldShape->getVertexAt(i + 1, j);
            Vector3 p4 = heightFieldShape->getVertexAt(i + 1, j + 1);

            p1 = transform * p1;
            p2 = transform * p2;
            p3 = transform * p3;
            p4 = transform * p4;

            mTriangles.add(DebugTriangle(p1, p2, p3, color));
			mTriangles.add(DebugTriangle(p3, p2, p4, color));
       }
   }
}

// Draw the collision shape of a collider
void DebugRenderer::drawCollisionShapeOfCollider(const Collider* collider, uint32 color) {
	
    switch (collider->getCollisionShape()->getName()) {
		
        case CollisionShapeName::BOX:
        {
            const BoxShape* boxShape = static_cast<const BoxShape*>(collider->getCollisionShape());
            drawBox(collider->getLocalToWorldTransform(), boxShape->getHalfExtents(), color);
            break;
        }
        case CollisionShapeName::SPHERE:
        {
            const SphereShape* sphereShape = static_cast<const SphereShape*>(collider->getCollisionShape());
            drawSphere(collider->getLocalToWorldTransform().getPosition(), sphereShape->getRadius(), color);
            break;
        }
        case CollisionShapeName::CAPSULE:
        {
            const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(collider->getCollisionShape());
            drawCapsule(collider->getLocalToWorldTransform(), capsuleShape->getRadius(), capsuleShape->getHeight(), color);
            break;
        }
        case CollisionShapeName::CONVEX_MESH:
        {
            const ConvexMeshShape*  convexMeshShape = static_cast<const ConvexMeshShape*>(collider->getCollisionShape());
            drawConvexMesh(collider->getLocalToWorldTransform(), convexMeshShape, color);
            break;
        }
        case CollisionShapeName::TRIANGLE_MESH:
        {
            const ConcaveMeshShape* concaveMeshShape = static_cast<const ConcaveMeshShape*>(collider->getCollisionShape());
            drawConcaveMeshShape(collider->getLocalToWorldTransform(), concaveMeshShape, color);
            break;
        }
        case CollisionShapeName::HEIGHTFIELD:
        {
            const HeightFieldShape* heighFieldShape = static_cast<const HeightFieldShape*>(collider->getCollisionShape());
            drawHeightFieldShape(collider->getLocalToWorldTransform(), heighFieldShape, color);
            break;
        }
        default:
        {
            assert(false);
        }
    }
}

// Generate the rendering primitives (triangles, lines, ...) of a physics world
void DebugRenderer::computeDebugRenderingPrimitives(const PhysicsWorld& world) {

	const bool drawColliderAABB = getIsDebugItemDisplayed(DebugItem::COLLIDER_AABB);
	const bool drawColliderBroadphaseAABB = getIsDebugItemDisplayed(DebugItem::COLLIDER_BROADPHASE_AABB);
	const bool drawCollisionShape = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE);
	
    const uint32 nbCollisionBodies = world.getNbCollisionBodies();
    const uint32 nbRigidBodies = world.getNbRigidBodies();

    // For each body of the world
    for (uint32 b = 0; b < nbCollisionBodies + nbRigidBodies; b++) {

		// Get a body
        const CollisionBody* body = b < nbCollisionBodies ? world.getCollisionBody(b) : world.getRigidBody(b - nbCollisionBodies);

        if (body->isActive()) {

            // For each collider of the body
            for (uint32 c = 0; c < body->getNbColliders(); c++) {

                // Get a collider
                const Collider* collider = body->getCollider(c);

                // If we need to draw the collider AABB
                if (drawColliderAABB) {

                    drawAABB(collider->getWorldAABB(), mMapDebugItemWithColor[DebugItem::COLLIDER_AABB]);
                }

                // If we need to draw the collider broad-phase AABB
                if (drawColliderBroadphaseAABB) {

                    if (collider->getBroadPhaseId() != -1) {
                        drawAABB(world.mCollisionDetection.mBroadPhaseSystem.getFatAABB(collider->getBroadPhaseId()), mMapDebugItemWithColor[DebugItem::COLLIDER_BROADPHASE_AABB]);
                    }
                }

                // If we need to draw the collision shape
                if (drawCollisionShape) {

                    drawCollisionShapeOfCollider(collider, mMapDebugItemWithColor[DebugItem::COLLISION_SHAPE]);
                }
            }
        }
    }
}

// Called when some contacts occur
void DebugRenderer::onContact(const CollisionCallback::CallbackData& callbackData) {

	// If we need to draw contact points
    if (getIsDebugItemDisplayed(DebugItem::CONTACT_POINT) || getIsDebugItemDisplayed(DebugItem::CONTACT_NORMAL)) {

		// For each contact pair
        for (uint32 p = 0; p < callbackData.getNbContactPairs(); p++) {

			CollisionCallback::ContactPair contactPair = callbackData.getContactPair(p);

            if (contactPair.getEventType() != CollisionCallback::ContactPair::EventType::ContactExit) {

                // For each contact point of the contact pair
                for (uint32 c = 0; c < contactPair.getNbContactPoints(); c++) {

                    CollisionCallback::ContactPoint contactPoint = contactPair.getContactPoint(c);

                    Vector3 point = contactPair.getCollider1()->getLocalToWorldTransform() * contactPoint.getLocalPointOnCollider1();

                    if (getIsDebugItemDisplayed(DebugItem::CONTACT_POINT)) {

                        // Contact point
                        drawSphere(point, mContactPointSphereRadius, mMapDebugItemWithColor[DebugItem::CONTACT_POINT]);
                    }

                    if (getIsDebugItemDisplayed(DebugItem::CONTACT_NORMAL)) {

                        // Contact normal
                        mLines.add(DebugLine(point,  point + contactPoint.getWorldNormal() * mContactNormalLength, mMapDebugItemWithColor[DebugItem::CONTACT_NORMAL]));
                    }
                }
            }
		}
	}
}
