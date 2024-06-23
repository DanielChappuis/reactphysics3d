/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
               mContactPointSphereRadius(DEFAULT_CONTACT_POINT_SPHERE_RADIUS), mContactNormalLength(DEFAULT_CONTACT_NORMAL_LENGTH),
               mCollisionShapeNormalLength(DEFAULT_COLLISION_SHAPE_NORMAL_LENGTH) {

    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLIDER_AABB, static_cast<uint32>(DebugColor::MAGENTA)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLIDER_BROADPHASE_AABB, static_cast<uint32>(DebugColor::YELLOW)));
	mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLISION_SHAPE, static_cast<uint32>(DebugColor::GREEN)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::CONTACT_POINT, static_cast<uint32>(DebugColor::RED)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::CONTACT_NORMAL, static_cast<uint32>(DebugColor::WHITE)));
    mMapDebugItemWithColor.add(Pair<DebugItem, uint32>(DebugItem::COLLISION_SHAPE_NORMAL, static_cast<uint32>(DebugColor::BLUE)));
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
void DebugRenderer::drawBox(const Transform& transform, const BoxShape* boxShape, uint32 colorShape, uint32 colorShapeNormals) {

    const bool drawShape = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE);
    const bool drawNormal = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE_NORMAL);

    Vector3 halfExtents = boxShape->getHalfExtents();
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

    if (drawShape) {

        // Triangle faces
        mTriangles.add(DebugTriangle(vertices[0], vertices[1], vertices[5], colorShape));
        mTriangles.add(DebugTriangle(vertices[0], vertices[5], vertices[4], colorShape));
        mTriangles.add(DebugTriangle(vertices[1], vertices[2], vertices[6], colorShape));
        mTriangles.add(DebugTriangle(vertices[1], vertices[6], vertices[5], colorShape));
        mTriangles.add(DebugTriangle(vertices[2], vertices[3], vertices[6], colorShape));
        mTriangles.add(DebugTriangle(vertices[3], vertices[7], vertices[6], colorShape));
        mTriangles.add(DebugTriangle(vertices[0], vertices[7], vertices[3], colorShape));
        mTriangles.add(DebugTriangle(vertices[0], vertices[4], vertices[7], colorShape));
        mTriangles.add(DebugTriangle(vertices[0], vertices[2], vertices[1], colorShape));
        mTriangles.add(DebugTriangle(vertices[0], vertices[3], vertices[2], colorShape));
        mTriangles.add(DebugTriangle(vertices[5], vertices[6], vertices[4], colorShape));
        mTriangles.add(DebugTriangle(vertices[4], vertices[6], vertices[7], colorShape));
    }

    if (drawNormal) {

        // Face normals
        for (int f=0; f < 6; f++) {

            const HalfEdgeStructure::Face& face = boxShape->getFace(f);
            const Vector3 facePoint = 0.25 * (boxShape->getVertexPosition(face.faceVertices[0]) + boxShape->getVertexPosition(face.faceVertices[1]) +
                         boxShape->getVertexPosition(face.faceVertices[2]) + boxShape->getVertexPosition(face.faceVertices[3]));
            const Vector3 normalPoint = facePoint + mCollisionShapeNormalLength * boxShape->getFaceNormal(f);

            mLines.add(DebugLine(transform * facePoint, transform * normalPoint, colorShapeNormals));
        }
    }

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
void DebugRenderer::drawConvexMesh(const Transform& transform, const ConvexMeshShape* convexMesh,
                                   uint32 colorShape, uint32 colorShapeNormals) {

    const bool drawShape = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE);
    const bool drawNormal = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE_NORMAL);

    // For each face of the convex mesh
	for (uint32 f = 0; f < convexMesh->getNbFaces(); f++) {

		const HalfEdgeStructure::Face& face = convexMesh->getFace(f);
		assert(face.faceVertices.size() >= 3);

        Vector3 centroid;

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

            if (v == 2) {
                centroid += v1 + v2;
            }

            centroid += v3;

            if (drawShape) {
                mTriangles.add(DebugTriangle(v1, v2, v3, colorShape));
            }
		}

        if (drawNormal) {

            centroid /= nbFaceVertices;
            const Vector3 normalPoint = centroid + transform.getOrientation() * convexMesh->getFaceNormal(f) * mCollisionShapeNormalLength;
            mLines.add(DebugLine(centroid, normalPoint, colorShapeNormals));
        }
    }
}

// Draw a concave mesh shape
void DebugRenderer::drawConcaveMeshShape(const Transform& transform, const ConcaveMeshShape* concaveMeshShape,
                                         uint32 colorShape, uint32 colorShapeNormals) {

    const bool drawShape = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE);
    const bool drawNormal = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE_NORMAL);

    if (drawShape) {

        // For each triangle of the mesh
        for (uint32 t = 0; t < concaveMeshShape->getNbTriangles(); t++) {

            Vector3 v1, v2, v3;
            concaveMeshShape->getTriangleVertices(t, v1, v2, v3);

            v1 = transform * v1;
            v2 = transform * v2;
            v3 = transform * v3;

            mTriangles.add(DebugTriangle(v1, v2, v3, colorShape));
        }
    }

    if (drawNormal) {

        // For each vertex of the mesh
        for (uint32 v = 0; v < concaveMeshShape->getNbVertices(); v++) {

            const Vector3& vertex = transform * concaveMeshShape->getVertex(v);
            const Vector3 normalPoint = vertex + transform.getOrientation() * concaveMeshShape->getVertexNormal(v) * mCollisionShapeNormalLength;
            mLines.add(DebugLine(vertex, normalPoint, colorShapeNormals));
        }
    }
}

// Draw a height field shape
void DebugRenderer::drawHeightFieldShape(const Transform& transform, const HeightFieldShape* heightFieldShape,
                                         uint32 colorShape, uint32 colorShapeNormals) {

    const bool drawShape = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE);
    const bool drawNormal = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE_NORMAL);

    // For each sub-grid points (except the last ones one each dimension)
    for (uint32 i = 0; i < heightFieldShape->getHeightField()->getNbColumns() - 1; i++) {
        for (uint32 j = 0; j < heightFieldShape->getHeightField()->getNbRows() - 1; j++) {

            // Compute the four point of the current quad
            Vector3 p1 = heightFieldShape->getVertexAt(i, j);
            Vector3 p2 = heightFieldShape->getVertexAt(i, j + 1);
            Vector3 p3 = heightFieldShape->getVertexAt(i + 1, j);
            Vector3 p4 = heightFieldShape->getVertexAt(i + 1, j + 1);

            p1 = transform * p1;
            p2 = transform * p2;
            p3 = transform * p3;
            p4 = transform * p4;

            if (drawShape) {

                mTriangles.add(DebugTriangle(p1, p2, p3, colorShape));
                mTriangles.add(DebugTriangle(p3, p2, p4, colorShape));
            }

            if (drawNormal) {

               // Compute the triangle normal
               Vector3 triangle1Normal = (p2 - p1).cross(p3 - p1).getUnit();

                const Vector3 normalPoint1 = p1 + triangle1Normal * mCollisionShapeNormalLength;
                const Vector3 normalPoint2 = p2 + triangle1Normal * mCollisionShapeNormalLength;
                const Vector3 normalPoint3 = p3 + triangle1Normal * mCollisionShapeNormalLength;
                const Vector3 normalPoint4 = p4 + triangle1Normal * mCollisionShapeNormalLength;
                mLines.add(DebugLine(p1, normalPoint1, colorShapeNormals));
                mLines.add(DebugLine(p2, normalPoint2, colorShapeNormals));
                mLines.add(DebugLine(p3, normalPoint3, colorShapeNormals));
                mLines.add(DebugLine(p4, normalPoint4, colorShapeNormals));
            }
       }
   }
}

// Draw the collision shape of a collider
void DebugRenderer::drawCollisionShapeOfCollider(const Collider* collider) {
	
    uint32 colorShape = mMapDebugItemWithColor[DebugItem::COLLISION_SHAPE];
    uint32 colorShapeNormals = mMapDebugItemWithColor[DebugItem::COLLISION_SHAPE_NORMAL];

    switch (collider->getCollisionShape()->getName()) {
		
        case CollisionShapeName::BOX:
        {
            const BoxShape* boxShape = static_cast<const BoxShape*>(collider->getCollisionShape());
            drawBox(collider->getLocalToWorldTransform(), boxShape, colorShape, colorShapeNormals);
            break;
        }
        case CollisionShapeName::SPHERE:
        {
            const SphereShape* sphereShape = static_cast<const SphereShape*>(collider->getCollisionShape());
            drawSphere(collider->getLocalToWorldTransform().getPosition(), sphereShape->getRadius(), colorShape);
            break;
        }
        case CollisionShapeName::CAPSULE:
        {
            const CapsuleShape* capsuleShape = static_cast<const CapsuleShape*>(collider->getCollisionShape());
            drawCapsule(collider->getLocalToWorldTransform(), capsuleShape->getRadius(), capsuleShape->getHeight(), colorShape);
            break;
        }
        case CollisionShapeName::CONVEX_MESH:
        {
            const ConvexMeshShape* convexMeshShape = static_cast<const ConvexMeshShape*>(collider->getCollisionShape());
            drawConvexMesh(collider->getLocalToWorldTransform(), convexMeshShape, colorShape, colorShapeNormals);
            break;
        }
        case CollisionShapeName::TRIANGLE_MESH:
        {
            const ConcaveMeshShape* concaveMeshShape = static_cast<const ConcaveMeshShape*>(collider->getCollisionShape());
            drawConcaveMeshShape(collider->getLocalToWorldTransform(), concaveMeshShape, colorShape, colorShapeNormals);
            break;
        }
        case CollisionShapeName::HEIGHTFIELD:
        {
            const HeightFieldShape* heighFieldShape = static_cast<const HeightFieldShape*>(collider->getCollisionShape());
            drawHeightFieldShape(collider->getLocalToWorldTransform(), heighFieldShape, colorShape, colorShapeNormals);
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
    const bool drawCollisionShapeNormals = getIsDebugItemDisplayed(DebugItem::COLLISION_SHAPE_NORMAL);

    const uint32 nbRigidBodies = world.getNbRigidBodies();

    // For each body of the world
    for (uint32 b = 0; b < nbRigidBodies; b++) {

		// Get a body
        const Body* body = world.getRigidBody(b);

        if (body->isActive() && body->isDebugEnabled()) {

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
                if (drawCollisionShape || drawCollisionShapeNormals) {

                    drawCollisionShapeOfCollider(collider);
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
