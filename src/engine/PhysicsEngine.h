/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

// Libraries
#include "PhysicsWorld.h"
#include "../collision/CollisionDetection.h"
#include "ConstraintSolver.h"
#include "../body/RigidBody.h"
#include "Timer.h"
#include "../configuration.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class PhysicsEngine :
        This class represents the physics engine
        of the library.
    -------------------------------------------------------------------
*/
class PhysicsEngine {
    private :
        PhysicsWorld* world;                            // Pointer to the physics world of the physics engine
        Timer timer;                                    // Timer of the physics engine
        CollisionDetection collisionDetection;          // Collision detection
        ConstraintSolver constraintSolver;              // Constraint solver
        bool isDeactivationActive;                      // True if the deactivation (sleeping) of inactive bodies is enabled

        void updateAllBodiesMotion();                                                                                                            // Compute the motion of all bodies and update their positions and orientations
        void updatePositionAndOrientationOfBody(RigidBody* body, const Vector3& newLinVelocity, const Vector3& newAngVelocity,
                                                const Vector3& linearVelocityErrorCorrection, const Vector3& angularVelocityErrorCorrection);    // Update the position and orientation of a body
        void setInterpolationFactorToAllBodies();                                                                                                // Compute and set the interpolation factor to all bodies
        void applyGravity();                                                                                                                     // Apply the gravity force to all bodies
        void resetBodiesMovementVariable();             // Reset the boolean movement variable of each body

public :
        PhysicsEngine(PhysicsWorld* world, decimal timeStep);    // Constructor
        ~PhysicsEngine();                                       // Destructor

        void start();                                                   // Start the physics simulation
        void stop();                                                    // Stop the physics simulation
        void update();                                                  // Update the physics simulation
        void setNbLCPIterations(uint nbIterations);                     // Set the number of iterations of the LCP solver
        void setIsErrorCorrectionActive(bool isErrorCorrectionActive);  // Set the isErrorCorrectionActive value

};

// --- Inline functions --- //

// Start the physics simulation
inline void PhysicsEngine::start() {
    timer.start();
}

inline void PhysicsEngine::stop() {
    timer.stop();
}                

// Set the number of iterations of the LCP solver
inline void PhysicsEngine::setNbLCPIterations(uint nbIterations) {
    constraintSolver.setNbLCPIterations(nbIterations);
}   

// Set the isErrorCorrectionActive value
inline void PhysicsEngine::setIsErrorCorrectionActive(bool isErrorCorrectionActive) {
    constraintSolver.setIsErrorCorrectionActive(isErrorCorrectionActive);
}

// Reset the boolean movement variable of each body
inline void PhysicsEngine::resetBodiesMovementVariable() {

    // For each rigid body
    for (std::set<RigidBody*>::iterator it = world->getRigidBodiesBeginIterator(); it != world->getRigidBodiesEndIterator(); it++) {

        // Set the hasMoved variable to false
        (*it)->setHasMoved(false);
    }
}

}

#endif
