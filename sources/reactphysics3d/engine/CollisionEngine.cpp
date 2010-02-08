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
#include <GL/freeglut.h>        // TODO : Remove this in the final version
#include <GL/gl.h>              // TODO : Remove this in the final version

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionEngine::CollisionEngine(PhysicsWorld* world, const Time& timeStep)
                   :DynamicEngine(world, timeStep) {
}

 // Destructor
CollisionEngine::~CollisionEngine() {

}

// Update the physics simulation
void CollisionEngine::update() {

    // While the time accumulator is not empty
    while(timer.getAccumulator() >= timer.getTimeStep().getValue()) {

        // Remove all old collision contact constraints
        world->removeAllContactConstraints();

        // Compute the collision detection
        if (collisionDetection.computeCollisionDetection(world)) {

            // TODO : Delete this ----------------------------------------------------------
            for (std::vector<Constraint*>::const_iterator it = world->getConstraintListStartIterator(); it != world->getConstraintListEndIterator(); ++it) {
                RigidBody* rigidBody1 = dynamic_cast<RigidBody*>((*it)->getBody1());
                RigidBody* rigidBody2 = dynamic_cast<RigidBody*>((*it)->getBody2());
                rigidBody1->setIsMotionEnabled(false);
                rigidBody2->setIsMotionEnabled(false);
            }
            // -----------------------------------------------------------------------------
        }

        // For each body in the dynamic world
        for(std::vector<Body*>::const_iterator it = world->getBodyListStartIterator(); it != world->getBodyListEndIterator(); ++it) {
            // If the body is a RigidBody and if the rigid body motion is enabled
            RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
            if (rigidBody && rigidBody->getIsMotionEnabled()) {
                // Update the state of the rigid body with an entire time step
                updateBodyState(rigidBody, timer.getTimeStep());
            }
        }

        // Update the timer
        timer.update();
    }

    // For each body in the the dynamic world
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
