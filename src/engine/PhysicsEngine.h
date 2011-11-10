/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

// Libraries
#include "PhysicsWorld.h"
#include "../collision/CollisionDetection.h"
#include "ConstraintSolver.h"
#include "../body/RigidBody.h"
#include "Timer.h"
#include "../constants.h"

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

        void updateAllBodiesMotion();                                                                                           // Compute the motion of all bodies and update their positions and orientations
        void updatePositionAndOrientationOfBody(RigidBody* body, const Vector3& newLinVelocity, const Vector3& newAngVelocity);    // Update the position and orientation of a body
        void setInterpolationFactorToAllBodies();                                                                                // Compute and set the interpolation factor to all bodies
        void applyGravity();                                                                                                    // Apply the gravity force to all bodies

public :
        PhysicsEngine(PhysicsWorld* world, double timeStep);    // Constructor
        ~PhysicsEngine();                                       // Destructor

        void start();                                           // Start the physics simulation
        void stop();                                            // Stop the physics simulation
        void update();                                          // Update the physics simulation
        void setPenetrationFactor(double factor);               // Set the penetration factor for the constraint solver
        void setNbLCPIterations(uint nbIterations);             // Set the number of iterations of the LCP solver
};

// --- Inline functions --- //

// Start the physics simulation
inline void PhysicsEngine::start() {
    timer.start();
}

inline void PhysicsEngine::stop() {
    timer.stop();
}      

// Set the penetration factor for the constraint solver
inline void PhysicsEngine::setPenetrationFactor(double factor) {
    constraintSolver.setPenetrationFactor(factor);
}               

// Set the number of iterations of the LCP solver
inline void PhysicsEngine::setNbLCPIterations(uint nbIterations) {
    constraintSolver.setNbLCPIterations(nbIterations);
}             

}

#endif
