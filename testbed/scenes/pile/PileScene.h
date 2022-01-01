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

#ifndef PILE_SCENE_H
#define PILE_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "SceneDemo.h"
#include "Sphere.h"
#include "Box.h"
#include "Capsule.h"
#include "ConvexMesh.h"
#include "ConcaveMesh.h"
#include "Dumbbell.h"
#include "VisualContactPoint.h"

namespace pilescene {

// Constants
const float SCENE_RADIUS = 30.0f;
const int NB_BOXES = 100;
const int NB_SPHERES = 40;
const int NB_CAPSULES = 30;
const int NB_MESHES = 30;
const int NB_COMPOUND_SHAPES = 0;
const openglframework::Vector3 BOX_SIZE(3, 3, 3);
const float SPHERE_RADIUS = 2.5f;
const float CAPSULE_RADIUS = 1.5f;
const float CAPSULE_HEIGHT = 3.0f;
const float DUMBBELL_HEIGHT = 1.0f;
const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50);        // Floor dimensions in meters

// Class PileScene
class PileScene : public SceneDemo {

    private :

        // -------------------- Attributes -------------------- //

        /// All the boxes of the scene
        std::vector<Box*> mBoxes;

        std::vector<Sphere*> mSpheres;

        std::vector<Capsule*> mCapsules;

        /// All the convex meshes of the scene
        std::vector<ConvexMesh*> mConvexMeshes;

        /// All the dumbbell of the scene
        std::vector<Dumbbell*> mDumbbells;

        /// Sandbox for the floor
        ConcaveMesh* mSandbox;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        PileScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~PileScene() override;

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
