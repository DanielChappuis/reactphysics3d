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

#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

// Libraries
#include "PhysicsWorld.h"
#include "../collision/CollisionDetection.h"
#include "ConstraintSolver.h"
#include "../body/RigidBody.h"
#include "Timer.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class PhysicsEngine :
        This class represents the physics engine
        of the library.
    -------------------------------------------------------------------
*/
class PhysicsEngine {
    protected :
        PhysicsWorld* world;                            // Pointer to the physics world of the physics engine
        Timer timer;                                    // Timer of the physics engine
        CollisionDetection collisionDetection;          // Collision detection
        ConstraintSolver constraintSolver;              // Constraint solver
                                                // Update the state of a rigid body // TODO : Delete this
        void updateAllBodiesMotion();                                                                                           // Compute the motion of all bodies and update their positions and orientations
        void updatePositionAndOrientationOfBody(Body* body, const Vector3D& newLinVelocity, const Vector3D& newAngVelocity);    // Update the position and orientation of a body
        void applyGravity();                                                                                                    // Apply the gravity force to all bodies
    public :
        PhysicsEngine(PhysicsWorld* world, const Time& timeStep) throw (std::invalid_argument);     // Constructor
        ~PhysicsEngine();                                                                           // Destructor

        void start();                                           // Start the physics simulation
        void stop();                                            // Stop the physics simulation
        void update();                                          // Update the physics simulation
        //void updateDynamic();                                   // TODO : Delete this method
        void updateCollision();                                 // TODO : Delete this collision
        void initializeDisplayTime(const Time& displayTime);    // Initialize the display time
        void updateDisplayTime(const Time& newDisplayTime);     // Update the display time
};

// --- Inline functions --- //

// Start the physics simulation
inline void PhysicsEngine::start() {
    timer.setIsRunning(true);
}

inline void PhysicsEngine::stop() {
    timer.setIsRunning(false);
}

// Initialize the display time
inline void PhysicsEngine::initializeDisplayTime(const Time& displayTime) {
    timer.setCurrentDisplayTime(displayTime);
}

// Update the display time
inline void PhysicsEngine::updateDisplayTime(const Time& newDisplayTime) {
    timer.updateDisplayTime(newDisplayTime);
}

}

#endif
