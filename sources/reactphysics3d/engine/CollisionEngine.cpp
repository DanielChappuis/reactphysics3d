/***************************************************************************
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
#include "CollisionEngine.h"
#include <cfloat>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionEngine::CollisionEngine(CollisionWorld* world, const Time& timeStep)
                   :DynamicEngine(world, timeStep) {

}

 // Destructor
CollisionEngine::~CollisionEngine() {

}

// Update the physics simulation
void CollisionEngine::update() {

    CollisionWorld* collisionWorld = dynamic_cast<CollisionWorld*>(world);
    assert(collisionWorld != 0);

    // While the time accumulator is not empty
    while(timer.getAccumulator() >= timer.getTimeStep().getValue()) {

        // Remove all old collision contact constraints
        collisionWorld->removeAllContactConstraints();

        Time timeFirst = 0;             // First collision time
        Time timeLast = DBL_MAX;        // Last collision separation time

        // Compute the collision detection
        if(collisionDetection.computeCollisionDetection(collisionWorld, timer.getTimeStep(), timeFirst, timeLast)) {
            // For each body in the dynamic world
            for(std::vector<Body*>::const_iterator it = world->getBodyListStartIterator(); it != world->getBodyListEndIterator(); ++it) {
                // If the body is a RigidBody and if the rigid body motion is enabled
                RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
                if (rigidBody && rigidBody->getIsMotionEnabled()) {
                    // Update the state of the rigid body
                    updateBodyState(rigidBody, timeFirst);
                }
            }
        }
        else {
            // For each body in the dynamic world
            for(std::vector<Body*>::const_iterator it = world->getBodyListStartIterator(); it != world->getBodyListEndIterator(); ++it) {
                // If the body is a RigidBody and if the rigid body motion is enabled
                RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
                if (rigidBody && rigidBody->getIsMotionEnabled()) {
                    // Update the state of the rigid body with an entire time step
                    updateBodyState(rigidBody, timer.getTimeStep());
                }
            }
        }

        // Update the timer
        timer.update();
    }

    // For each body in the dynamic world
    for(std::vector<Body*>::const_iterator it = world->getBodyListStartIterator(); it != world->getBodyListEndIterator(); ++it) {
        // If the body is a RigidBody and if the rigid body motion is enabled
        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        if (rigidBody && rigidBody->getIsMotionEnabled()) {
            // Update the interpolation factor of the rigid body
            // This one will be used to compute the interpolated state
            rigidBody->setInterpolationFactor(timer.getInterpolationFactor());
        }
    }
}
