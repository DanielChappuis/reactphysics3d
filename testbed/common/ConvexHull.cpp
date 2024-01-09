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

// Libraries
#include "ConvexHull.h"
#include <unordered_set>
#include <reactphysics3d/utils/Message.h>

// Constructor
ConvexHull::ConvexHull(reactphysics3d::BodyType type, bool isSimulationCollider, rp3d::PhysicsCommon& physicsCommon, rp3d::PhysicsWorld* physicsWorld,
                       const std::string& meshPath, const reactphysics3d::Vector3& scaling)
           : ConvexMesh(physicsCommon, physicsWorld, meshPath) {

    // Compute the scaling matrix
    mScalingMatrix = openglframework::Matrix4(scaling.x, 0, 0, 0,
                                              0, scaling.y, 0, 0,
                                              0, 0, scaling.z, 0,
                                              0, 0, 0, 1);

    // Vertex array with all vertices of the triangle mesh
    rp3d::VertexArray vertexArray(&(mVertices[0]), sizeof(openglframework::Vector3),
                                  mVertices.size(), rp3d::VertexArray::DataType::VERTEX_FLOAT_TYPE);

    // Create the convex mesh
    std::vector<rp3d::Message> messages;
    mConvexMesh = mPhysicsCommon.createConvexMesh(vertexArray, messages);
    if (mConvexMesh == nullptr) {
        std::cout << "Error while creating a ConvexMesh:" << std::endl;
        for (const rp3d::Message& message: messages) {
            std::cout << "Error: " << message.text << std::endl;
        }
    }

    // Create the collision shape for the rigid body (convex mesh shape) and do
    // not forget to delete it at the end
    mConvexShape = mPhysicsCommon.createConvexMeshShape(mConvexMesh, scaling);

    mPreviousTransform = rp3d::Transform::identity();

    // Create a rigid body corresponding in the physics world
    rp3d::RigidBody* body = physicsWorld->createRigidBody(mPreviousTransform);
    body->setType(type);
    body->setIsDebugEnabled(true);
    mCollider = body->addCollider(mConvexShape, rp3d::Transform::identity());
    mCollider->setIsSimulationCollider(isSimulationCollider);
    body->updateMassPropertiesFromColliders();
    mBody = body;

    mTransformMatrix = mTransformMatrix * mScalingMatrix;
}
