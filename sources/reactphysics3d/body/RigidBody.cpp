/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include "RigidBody.h"

 // We want to use the ReactPhysics3D namespace
 using namespace reactphysics3d;

 // Constructor
 RigidBody::RigidBody(const Vector3D& position, const Quaternion& orientation, double mass, const Matrix3x3& inertiaTensorLocal, BoundingVolume* broadBoundingVolume,
                      BoundingVolume* narrowBoundingVolume)
           : Body(mass, broadBoundingVolume, narrowBoundingVolume), position(position), orientation(orientation.getUnit()), inertiaTensorLocal(inertiaTensorLocal),
             inertiaTensorLocalInverse(inertiaTensorLocal.getInverse()), massInverse(1.0/mass), oldPosition(position), oldOrientation(orientation) {

    restitution = 1.0;
    isMotionEnabled = true;
    isCollisionEnabled = true;
    interpolationFactor = 0.0;

    // Update the orientation of the OBB according to the orientation of the rigid body
    update();

    assert(broadBoundingVolume);
    assert(narrowBoundingVolume);
    
    // Set the body pointer to the bounding volumes
    // TODO : Move this in the Body constructor
    broadBoundingVolume->setBodyPointer(this);
    narrowBoundingVolume->setBodyPointer(this);
}

// Destructor
RigidBody::~RigidBody() {

};