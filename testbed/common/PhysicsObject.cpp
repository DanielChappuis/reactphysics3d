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
#include "PhysicsObject.h"

/// Constructor
PhysicsObject::PhysicsObject(rp3d::PhysicsCommon& physicsCommon)
              : openglframework::Mesh(), mPhysicsCommon(physicsCommon) {

    mBody = nullptr;
    mColor = openglframework::Color(1, 1, 1, 1);
    mSleepingColor = openglframework::Color(1, 0, 0, 1);
}

/// Constructor
PhysicsObject::PhysicsObject(rp3d::PhysicsCommon& physicsCommon, const std::string& meshPath) : PhysicsObject(physicsCommon) {

    // Load the mesh from a file
    openglframework::MeshReaderWriter::loadMeshFromFile(meshPath, *this);

	// If the mesh file do not have normals
	if (mNormals.empty()) {

		// Calculate the normals of the mesh
		calculateNormals();
	}
}

// Compute the new transform matrix
openglframework::Matrix4 PhysicsObject::computeTransform(float interpolationFactor,
                                                        const openglframework::Matrix4& scalingMatrix) {

    // Get the transform of the rigid body
    rp3d::Transform transform = mBody->getTransform();

    // Interpolate the transform between the previous one and the new one
    rp3d::Transform interpolatedTransform = rp3d::Transform::interpolateTransforms(mPreviousTransform,
                                                                                  transform,
                                                                                  interpolationFactor);
    mPreviousTransform = transform;

    // Compute the transform used for rendering the box
    rp3d::decimal matrix[16];
    interpolatedTransform.getOpenGLMatrix(matrix);
    openglframework::Matrix4 newMatrix(matrix[0], matrix[4], matrix[8], matrix[12],
                                       matrix[1], matrix[5], matrix[9], matrix[13],
                                       matrix[2], matrix[6], matrix[10], matrix[14],
                                       matrix[3], matrix[7], matrix[11], matrix[15]);

    // Apply the scaling matrix to have the correct box dimensions
    return newMatrix * scalingMatrix;
}

// Reset the transform
void PhysicsObject::setTransform(const rp3d::Transform& transform) {

    // Reset the transform
    mBody->setTransform(transform);

    // Reset the velocity of the rigid body
    rp3d::RigidBody* rigidBody = dynamic_cast<rp3d::RigidBody*>(mBody);
    if (rigidBody != nullptr) {
        rigidBody->setLinearVelocity(rp3d::Vector3(0, 0, 0));
        rigidBody->setAngularVelocity(rp3d::Vector3(0, 0, 0));
    }

    updateTransform(1.0f);
}
