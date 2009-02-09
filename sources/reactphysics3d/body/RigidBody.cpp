/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

 // Libraries
 #include "RigidBody.h"

 // We want to use the ReactPhysics3D namespace
 using namespace reactphysics3d;

 // Constructor
 RigidBody::RigidBody(const Vector3D& position, const Kilogram& mass, const Matrix3x3& inertiaTensor)
           : Body(mass), inertiaTensor(inertiaTensor),
             currentBodyState(position, inertiaTensor.getInverse(),Kilogram(1.0/mass.getValue())),
             previousBodyState(position, inertiaTensor.getInverse(), Kilogram(1.0/mass.getValue())) {
        isMotionEnabled = true;
        isCollisionEnabled = true;
 }

 // Copy-constructor
 RigidBody::RigidBody(const RigidBody& rigidBody) : Body(rigidBody), inertiaTensor(rigidBody.inertiaTensor),
            currentBodyState(rigidBody.currentBodyState), previousBodyState(rigidBody.previousBodyState) {
    this->isMotionEnabled = rigidBody.isMotionEnabled;
    this->isCollisionEnabled = rigidBody.isCollisionEnabled;
 }

 // Destructor
 RigidBody::~RigidBody() {

 };

// Return the force on the body at time t
Vector3D RigidBody::computeForce(Time time) const {
    // TODO : Implement this method
    return Vector3D(1*time.getValue(),0,0);
}

// Return the torque on the body at time
Vector3D RigidBody::computeTorque(Time time) const {
    // TODO : Implement this method
    return Vector3D(1*time.getValue(),0,0);
}
