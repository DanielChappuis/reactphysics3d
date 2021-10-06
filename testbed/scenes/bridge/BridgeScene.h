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

#ifndef BRIDGE_SCENE_H
#define BRIDGE_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "Sphere.h"
#include "SceneDemo.h"

namespace bridgescene {

// Constants
const float SCENE_RADIUS = 60.0f;
const float SPHERE_RADIUS = 2.0f;
const float SPHERE_MASS = 40.0f;
const openglframework::Vector3 BOX_SIZE = openglframework::Vector3(2, 0.5, 4);
const int NB_BRIDGES = 4;
const int NB_BOXES = 16;

// Class BridgeScene scene
class BridgeScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Boxes
        Box* mBoxes[NB_BOXES * NB_BRIDGES];

        /// Spheres
        Sphere* mSpheres[NB_BRIDGES];

        /// Hinge joints of the bridge
        std::vector<rp3d::HingeJoint*> mHingeJoints;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create the joints
        void createJoints();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        BridgeScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~BridgeScene() override ;

        /// Update the physics simulation
        virtual void updatePhysics() override;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// Initialize the bodies positions
        void initBodiesPositions();
};

}

#endif
